const fs = require("node:fs");
const crypto = require("node:crypto");
const http = require("node:http");
const path = require("node:path");
const { chromium } = require(path.join(
  process.env.RUNNER_TEMP,
  "web-smoke",
  "node_modules",
  "playwright"
));

const siteRoot = path.resolve("_site");
const mimeTypes = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".wasm": "application/wasm",
};
const server = http.createServer((request, response) => {
  const requestPath = new URL(request.url, "http://127.0.0.1").pathname;
  const relativePath = decodeURIComponent(requestPath).replace(/^\/+/, "");
  if (relativePath === "favicon.ico") {
    response.writeHead(204).end();
    return;
  }
  const filePath = path.resolve(siteRoot, relativePath || "index.html");
  if (filePath !== siteRoot && !filePath.startsWith(`${siteRoot}${path.sep}`)) {
    response.writeHead(403).end();
    return;
  }

  fs.createReadStream(filePath)
    .on("error", () => response.writeHead(404).end())
    .on("open", () => response.writeHead(200, {
      "Content-Type": mimeTypes[path.extname(filePath)] || "application/octet-stream",
    }))
    .pipe(response);
});

async function run() {
  let browser;
  try {
    await new Promise((resolve, reject) => {
      server.once("error", reject);
      server.listen(4173, "127.0.0.1", resolve);
    });
    browser = await chromium.launch({
      headless: true,
      channel: "chrome",
      args: [
        "--use-gl=angle",
        "--use-angle=swiftshader",
        "--enable-webgl",
        "--enable-unsafe-swiftshader",
      ],
    });
    const context = await browser.newContext({ acceptDownloads: true });
    const page = await context.newPage();
    page.on("console", (message) => {
      if (message.type() === "error")
        console.error(`브라우저 오류: ${message.text()} (${JSON.stringify(message.location())})`);
    });
    page.on("pageerror", (error) => {
      console.error(`페이지 오류: ${error.stack || error.message} [${error.name || typeof error}]`);
    });
    page.on("requestfailed", (request) => {
      console.error(`요청 실패: ${request.url()} (${request.failure()?.errorText || "원인 없음"})`);
    });
    await page.addInitScript(() => {
      window.addEventListener("error", (event) => {
        console.error("window 오류 위치:", event.message, event.filename,
          event.lineno, event.colno, event.error?.stack || String(event.error));
      });
      window.addEventListener("unhandledrejection", (event) => {
        console.error("처리되지 않은 비동기 오류:", event.reason?.stack || String(event.reason));
      });
    });
    await page.goto("http://127.0.0.1:4173/", {
      waitUntil: "commit",
      timeout: 15000,
    });
    await page.waitForFunction(
      () => {
        const status = document.querySelector("#startup-status");
        return status?.hidden === true || status?.getAttribute("role") === "alert";
      },
      null,
      { timeout: 45000 }
    );
    const startupError = await page.evaluate(() => {
      const status = document.querySelector("#startup-status[role='alert']");
      return status?.textContent?.trim() || null;
    });
    if (startupError)
      throw new Error(startupError.trim());

    const telemetry = page.locator("#web-gcs-status");
    await page.waitForFunction(
      () => {
        const status = document.querySelector("#web-gcs-status")?.textContent || "";
        return status.includes("비행 상태:")
          && status.includes("가상 시험면 촬영 조건:");
      },
      null,
      { timeout: 5000 }
    );
    await page.waitForFunction(
      () => {
        const video = document.querySelector("#web-eo-live-video");
        const streamStatus = document.querySelector("#web-eo-live-status")?.textContent || "";
        return streamStatus.includes("로컬 WebRTC 수신 연결됨")
          && video?.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA
          && video.videoWidth > 0 && video.videoHeight > 0
          && video.srcObject instanceof MediaStream
          && video.srcObject.getVideoTracks().some((track) => track.readyState === "live");
      },
      null,
      { timeout: 20000 }
    );
    console.log("Chromium에서 EO 영상의 WebRTC 송수신과 수신 화면을 확인했습니다.");
    await page.getByRole("button", { name: "이륙" }).click();
    await page.waitForFunction(
      () => !document.querySelector("#web-gcs-status")?.textContent.includes("grounded"),
      null,
      { timeout: 10000 }
    );
    const takeoffState = await telemetry.innerText();
    if (!takeoffState.includes("taking_off") && !takeoffState.includes("hovering")
        && !takeoffState.includes("flying"))
      throw new Error(`이륙 버튼 뒤 비행 상태가 바뀌지 않았습니다: ${takeoffState}`);
    await page.waitForFunction(
      () => document.querySelector("#web-gcs-status")?.textContent.includes("hovering"),
      null,
      { timeout: 10000 }
    );

    const forwardButton = page.getByRole("button", { name: "전진" });
    await forwardButton.hover();
    await page.mouse.down();
    await page.waitForFunction(
      () => {
        const status = document.querySelector("#web-gcs-status")?.textContent || "";
        const speed = Number(status.match(/속도 ([0-9.]+) m\/s/)?.[1] || 0);
        return status.includes("비행 상태: flying") && speed > 0.1;
      },
      null,
      { timeout: 5000 }
    );
    await page.waitForTimeout(250);
    await page.mouse.up();
    await page.getByRole("button", { name: "착륙" }).click();
    await page.waitForFunction(
      () => document.querySelector("#web-gcs-status")?.textContent.includes("landing"),
      null,
      { timeout: 5000 }
    );

    const responsiveness = await page.evaluate(() => new Promise((resolve) => {
      const frameTimes = [];
      const startedAt = performance.now();
      function recordFrame(now) {
        frameTimes.push(now);
        if (now - startedAt >= 5000) {
          const gaps = frameTimes.slice(1).map((time, index) => time - frameTimes[index]);
          resolve({
            frameCount: frameTimes.length,
            maximumFrameGapMilliseconds: gaps.length ? Math.max(...gaps) : 0,
          });
          return;
        }
        requestAnimationFrame(recordFrame);
      }
      requestAnimationFrame(recordFrame);
    }));
    if (responsiveness.frameCount < 30
        || responsiveness.maximumFrameGapMilliseconds > 1500)
    {
      throw new Error(
        `첫 화면 뒤 갱신이 멈췄습니다. ${responsiveness.frameCount}프레임, `
        + `최대 공백 ${Math.round(responsiveness.maximumFrameGapMilliseconds)}ms`);
    }
    console.log(
      `Chromium에서 첫 화면, 비행 조작, 상태 갱신을 확인했습니다. `
      + `${responsiveness.frameCount}프레임, 최대 공백 `
      + `${Math.round(responsiveness.maximumFrameGapMilliseconds)}ms`);

    await page.reload({ waitUntil: "domcontentloaded" });
    await page.waitForFunction(
      () => document.querySelector("#startup-status")?.hidden === true,
      null,
      { timeout: 45000 }
    );
    await page.waitForFunction(
      () => document.querySelector("#web-gcs-status")?.textContent.includes("비행 상태: grounded"),
      null,
      { timeout: 10000 }
    );
    await page.waitForFunction(
      () => document.querySelector("#inspection-target")?.options.length === 4,
      null,
      { timeout: 10000 }
    );
    const inspectionTarget = page.locator("#inspection-target");
    await inspectionTarget.selectOption("1");
    const selectedFacility = await inspectionTarget.locator("option:checked").innerText();
    await page.getByRole("button", { name: "자동 점검 시작" }).click();
    try {
      await page.waitForFunction(
        () => {
          const status = document.querySelector("#web-gcs-status")?.textContent || "";
          return status.includes("점검 임무: 구도 확인 완료")
            || status.includes("점검 임무: 실패:");
        },
        null,
        { timeout: 240000 }
      );
    } catch {
      throw new Error(`자동 점검 임무가 끝나지 않았습니다: ${await telemetry.innerText()}`);
    }
    const missionStatus = await telemetry.innerText();
    if (!missionStatus.includes("점검 임무: 구도 확인 완료 (1개 구역)"))
      throw new Error(`자동 점검 임무가 끝나지 않았습니다: ${missionStatus}`);
    await page.waitForFunction(
      () => {
        const text = document.querySelector("#web-eo-recording")?.textContent || "";
        return text.includes("내려받을 수 있습니다")
          || text.includes("브라우저에 저장했습니다");
      },
      null,
      { timeout: 10000 }
    );

    const videoDownloadPromise = page.waitForEvent("download");
    await page.getByRole("link", { name: "영상 내려받기", exact: true }).click();
    const videoDownload = await videoDownloadPromise;
    if (!videoDownload.suggestedFilename().endsWith(".webm"))
      throw new Error(`영상 파일 형식이 WebM이 아닙니다: ${videoDownload.suggestedFilename()}`);
    const videoPath = await videoDownload.path();
    if (!videoPath || fs.statSync(videoPath).size === 0)
      throw new Error("EO 영상 파일이 비어 있습니다.");

    const manifestDownloadPromise = page.waitForEvent("download");
    await page.getByRole("link", { name: "촬영 기록 내려받기" }).click();
    const manifestDownload = await manifestDownloadPromise;
    const manifestPath = await manifestDownload.path();
    if (!manifestPath)
      throw new Error("촬영 기록 파일을 내려받지 못했습니다.");
    const manifest = JSON.parse(fs.readFileSync(manifestPath, "utf8"));
    const videoBytes = fs.readFileSync(videoPath);
    const videoHash = crypto.createHash("sha256").update(videoBytes).digest("hex");
    if (manifest.format_version !== 4
        || manifest.collection_state !== "CAPTURED"
        || manifest.verification_state !== "VERIFIED"
        || manifest.storage_state !== "SAVED"
        || manifest.storage_backend !== "OPFS"
        || manifest.mission_outcome !== "COMPLETED"
        || manifest.video_bytes !== videoBytes.length
        || manifest.video_checksum !== videoHash
        || manifest.recording_duration_milliseconds <= 0
        || manifest.sample_interval_milliseconds !== 250
        || !Array.isArray(manifest.samples) || manifest.samples.length < 2
        || !Array.isArray(manifest.expected_zones) || manifest.expected_zones.length !== 1
        || !Array.isArray(manifest.captured_zones) || manifest.captured_zones.length !== 1
        || !Array.isArray(manifest.missing_zones) || manifest.missing_zones.length !== 0
        || !Array.isArray(manifest.missing_zone_videos)
        || manifest.missing_zone_videos.length !== 0
        || manifest.capture_summary?.expected_zone_count !== 1
        || manifest.capture_summary?.captured_zone_count !== 1
        || manifest.capture_summary?.missing_zone_count !== 0
        || !Array.isArray(manifest.zone_recordings)
        || manifest.zone_recordings.length !== 1
        || manifest.zone_recordings[0].collection_state !== "CAPTURED"
        || manifest.zone_recordings[0].storage_state !== "SAVED"
        || manifest.zone_recordings[0].storage_backend !== "OPFS"
        || manifest.zone_recordings[0].verification_state !== "VERIFIED"
        || manifest.zone_recordings[0].download_ready !== true
        || manifest.zone_recordings[0].video_bytes <= 0
        || !manifest.zone_recordings[0].video_checksum)
      throw new Error("촬영 기록의 영상 크기·체크섬·상태 표본이 올바르지 않습니다.");
    const capturedZone = manifest.captured_zones[0];
    if (capturedZone.inspection_zone_id !== manifest.expected_zones[0].inspection_zone_id
        || !Number.isFinite(capturedZone.simulation_tick)
        || !Number.isFinite(capturedZone.simulation_time_seconds)
        || capturedZone.video_frame_requested !== true
        || capturedZone.recording_elapsed_milliseconds < 0
        || capturedZone.collection_state !== "CAPTURED"
        || capturedZone.storage_state !== "SAVED"
        || capturedZone.verification_state !== "VERIFIED"
        || capturedZone.download_ready !== true
        || !capturedZone.video_file)
      throw new Error("완료된 점검 구역의 실제 촬영 프레임 시각이 기록되지 않았습니다.");
    const zoneVideoPromise = page.waitForEvent("download");
    await page.locator("#web-eo-zone-downloads a[data-zone-video]").click();
    const zoneVideoDownload = await zoneVideoPromise;
    const zoneVideoPath = await zoneVideoDownload.path();
    if (!zoneVideoPath || !zoneVideoDownload.suggestedFilename().endsWith(".webm"))
      throw new Error("점검 구역별 영상을 내려받지 못했습니다.");
    const zoneVideoBytes = fs.readFileSync(zoneVideoPath);
    const zoneVideoHash = crypto.createHash("sha256").update(zoneVideoBytes).digest("hex");
    const savedZoneVideo = manifest.zone_recordings[0];
    if (zoneVideoBytes.length !== savedZoneVideo.video_bytes
        || zoneVideoHash !== savedZoneVideo.video_checksum
        || savedZoneVideo.video_file !== capturedZone.video_file
        || savedZoneVideo.inspection_zone_id !== capturedZone.inspection_zone_id
        || !Number.isFinite(savedZoneVideo.start_simulation_tick)
        || savedZoneVideo.capture_simulation_tick
          - savedZoneVideo.start_simulation_tick < 60)
      throw new Error("구역별 영상과 촬영 기록의 크기·체크섬·구역 ID가 맞지 않습니다.");
    for (let index = 1; index < manifest.samples.length; ++index) {
      const elapsedDifference = manifest.samples[index].recording_elapsed_milliseconds
        - manifest.samples[index - 1].recording_elapsed_milliseconds;
      if (elapsedDifference < 200)
        throw new Error("촬영 상태 표본이 250ms보다 짧은 간격으로 중복 저장됐습니다.");
    }
    const inspectedZones = new Set(manifest.samples
      .map((entry) => entry.inspection_zone_id)
      .filter((zoneId) => typeof zoneId === "string" && zoneId.length > 0));
    if (inspectedZones.size !== 1)
      throw new Error(`선택한 점검 구역만 기록되지 않았습니다: ${[...inspectedZones]}`);
    const capturedFacilities = new Set(manifest.samples
      .map((entry) => entry.facility_id)
      .filter((facilityId) => typeof facilityId === "string" && facilityId.length > 0));
    if (capturedFacilities.size !== 1 || !selectedFacility.includes([...capturedFacilities][0]))
      throw new Error(`선택한 OSM 건물만 촬영하지 않았습니다: ${selectedFacility}`);
    const sample = manifest.samples.find((entry) => entry.inspection_zone_id);
    if (!sample || !Array.isArray(sample.drone_position_meters)
        || !Array.isArray(sample.camera_position_meters)
        || !Array.isArray(sample.camera_orientation_xyzw)
        || !Number.isFinite(sample.simulation_time_seconds)
        || !sample.capture_assessment)
      throw new Error("촬영 기록에 점검 구역·시간·위치·자세·판정 정보가 없습니다.");
    console.log("Chromium에서 점검 구역별 영상, 임무 영상, 촬영 기록과 체크섬을 확인했습니다.");

    const expectedBrowserFiles = [
      manifest.video_file,
      manifest.video_file.replace(/\.webm$/i, ".manifest.json"),
      savedZoneVideo.video_file,
    ];
    await page.reload({ waitUntil: "domcontentloaded" });
    await page.waitForFunction(
      () => document.querySelector("#startup-status")?.hidden === true,
      null,
      { timeout: 45000 }
    );
    await page.waitForFunction(
      (expectedNames) => {
        const savedNames = [...document.querySelectorAll("#web-eo-saved-files a[data-opfs-file]")]
          .map((link) => link.dataset.opfsFile);
        return expectedNames.every((name) => savedNames.includes(name));
      },
      expectedBrowserFiles,
      { timeout: 10000 }
    );
    console.log("새로고침 뒤에도 OPFS의 임무 영상·구역 영상·촬영 기록을 다시 찾았습니다.");
  } catch (error) {
    console.error(`웹 시작 검사 실패: ${error.message}`);
    process.exitCode = 1;
  } finally {
    if (browser)
      await browser.close();
    await new Promise((resolve) => server.close(resolve));
  }
}

run();
