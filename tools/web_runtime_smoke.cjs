const fs = require("node:fs");
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

function summarizeCpuProfile(profile) {
  const frames = new Map(profile.nodes.map((node) => [node.id, node.callFrame]));
  const totals = new Map();
  for (let index = 0; index < profile.samples.length; index += 1) {
    const frame = frames.get(profile.samples[index]);
    if (!frame)
      continue;
    const name = frame.functionName || "(anonymous)";
    const source = frame.url ? path.basename(frame.url) : "browser runtime";
    const key = `${source}:${frame.lineNumber + 1} ${name}`;
    totals.set(key, (totals.get(key) || 0) + (profile.timeDeltas[index] || 0));
  }
  return [...totals.entries()]
    .sort((first, second) => second[1] - first[1])
    .slice(0, 12)
    .map(([frame, microseconds]) => ({
      frame,
      sampledMilliseconds: Math.round(microseconds / 100) / 10,
    }));
}

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
    const webGlErrors = [];
    page.on("console", (message) => {
      if (message.type() === "error") {
        console.error(`브라우저 오류: ${message.text()} (${JSON.stringify(message.location())})`);
        if (message.text().includes("GL_INVALID_OPERATION"))
          webGlErrors.push(message.text());
      }
    });
    page.on("pageerror", (error) => {
      console.error(`페이지 오류: ${error.stack || error.message} [${error.name || typeof error}]`);
    });
    page.on("requestfailed", (request) => {
      console.error(`요청 실패: ${request.url()} (${request.failure()?.errorText || "원인 없음"})`);
    });
    await page.addInitScript(() => {
      window.__droneSimFrameTimings = [];
      window.addEventListener("error", (event) => {
        const startupStatus = document.querySelector("#startup-status");
        const startupMessage = document.querySelector("#startup-message");
        console.error("window 오류 위치:", event.message, event.filename,
          event.lineno, event.colno, event.error?.stack || String(event.error),
          "초기화 단계:", startupMessage?.textContent || "상태 없음",
          "상태 역할:", startupStatus?.getAttribute("role") || "없음");
      });
      window.addEventListener("unhandledrejection", (event) => {
        console.error("처리되지 않은 비동기 오류:", event.reason?.stack || String(event.reason));
      });
    });
    await page.goto("http://127.0.0.1:4173/?frame-probe=1", {
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
    await page.waitForTimeout(100);
    if (webGlErrors.length > 0)
      throw new Error(`WebGL 초기화 중 그래픽 오류가 발생했습니다: ${webGlErrors.join(" | ")}`);

    const telemetry = page.locator("#web-gcs-status");
    await page.waitForFunction(
      () => {
        return document.querySelector("#web-gcs-mode")?.textContent === "grounded"
          && document.querySelector("#web-gcs-capture")?.textContent
            .includes("파손 여부는 사람이 판단합니다.");
      },
      null,
      { timeout: 5000 }
    );
    await page.waitForFunction(
      () => {
        const canvas = document.querySelector("#web-eo-live-canvas");
        const streamStatus = document.querySelector("#web-eo-live-status")?.textContent || "";
        if (!streamStatus.includes("카메라 화면을 표시")
            || !canvas || canvas.width <= 0 || canvas.height <= 0)
          return false;
        const context = canvas.getContext("2d", { willReadFrequently: true });
        const pixels = context.getImageData(0, 0,
          Math.min(32, canvas.width), Math.min(24, canvas.height)).data;
        for (let index = 0; index < pixels.length; index += 4) {
          if (pixels[index] > 20 || pixels[index + 1] > 20 || pixels[index + 2] > 20)
            return true;
        }
        return false;
      },
      null,
      { timeout: 20000 }
    );
    const cameraPixels = await page.evaluate(() => {
      const source = document.querySelector("#web-eo-live-canvas");
      const canvas = document.createElement("canvas");
      canvas.width = 32;
      canvas.height = 24;
      const context = canvas.getContext("2d", { willReadFrequently: true });
      context.drawImage(source, 0, 0, canvas.width, canvas.height);
      const pixels = context.getImageData(0, 0, canvas.width, canvas.height).data;
      const colors = new Set();
      let visiblePixels = 0;
      for (let index = 0; index < pixels.length; index += 4) {
        const red = pixels[index];
        const green = pixels[index + 1];
        const blue = pixels[index + 2];
        if (red > 20 || green > 20 || blue > 20)
          visiblePixels += 1;
        colors.add(`${red >> 4},${green >> 4},${blue >> 4}`);
      }
      return { visiblePixels, colorCount: colors.size };
    });
    if (cameraPixels.visiblePixels < 24 || cameraPixels.colorCount < 8)
      throw new Error(`EO 카메라 패널이 비어 있습니다: ${JSON.stringify(cameraPixels)}`);
    console.log("Chromium에서 같은 페이지의 EO 카메라 패널 표시를 확인했습니다.");
    await page.getByRole("button", { name: "이륙" }).click();
    await page.waitForFunction(
      () => document.querySelector("#web-gcs-mode")?.textContent !== "grounded",
      null,
      { timeout: 10000 }
    );
    const takeoffState = await telemetry.innerText();
    if (!takeoffState.includes("taking_off") && !takeoffState.includes("hovering")
        && !takeoffState.includes("flying"))
      throw new Error(`이륙 버튼 뒤 비행 상태가 바뀌지 않았습니다: ${takeoffState}`);
    await page.waitForFunction(
      () => document.querySelector("#web-gcs-mode")?.textContent.includes("hovering"),
      null,
      { timeout: 10000 }
    );

    const forwardButton = page.getByRole("button", { name: "전진" });
    await forwardButton.hover();
    await page.mouse.down();
    await page.waitForFunction(
      () => {
        const mode = document.querySelector("#web-gcs-mode")?.textContent || "";
        const speed = Number.parseFloat(
          document.querySelector("#web-gcs-speed")?.textContent || "0");
        return mode === "flying" && speed > 0.1;
      },
      null,
      { timeout: 5000 }
    );
    await page.waitForTimeout(250);
    await page.mouse.up();
    await page.getByRole("button", { name: "착륙" }).click();
    await page.waitForFunction(
      () => document.querySelector("#web-gcs-mode")?.textContent.includes("landing"),
      null,
      { timeout: 5000 }
    );

    const responsivenessSession = process.env.WEB_SMOKE_CPU_PROFILE === "1"
      ? await context.newCDPSession(page)
      : null;
    if (responsivenessSession) {
      await responsivenessSession.send("Profiler.enable");
      await responsivenessSession.send("Profiler.setSamplingInterval", { interval: 1000 });
      await responsivenessSession.send("Profiler.start");
    }
    const responsiveness = await page.evaluate(() => new Promise((resolve) => {
      const frameTimes = [];
      const longTasks = [];
      const longTaskObserver = PerformanceObserver.supportedEntryTypes.includes("longtask")
        ? new PerformanceObserver((list) => {
          for (const entry of list.getEntries()) {
            longTasks.push({
              durationMilliseconds: Math.round(entry.duration * 10) / 10,
              startMilliseconds: Math.round(entry.startTime),
              source: entry.attribution?.[0]?.containerName || entry.name,
            });
          }
        })
        : null;
      longTaskObserver?.observe({ type: "longtask", buffered: true });
      const startedAt = performance.now();
      function recordFrame(now) {
        frameTimes.push(now);
        if (now - startedAt >= 5000) {
          const gaps = frameTimes.slice(1).map((time, index) => time - frameTimes[index]);
          longTaskObserver?.disconnect();
          resolve({
            frameCount: frameTimes.length,
            maximumFrameGapMilliseconds: gaps.length ? Math.max(...gaps) : 0,
            longestTasks: longTasks
              .sort((first, second) => second.durationMilliseconds
                - first.durationMilliseconds)
              .slice(0, 8),
          });
          return;
        }
        requestAnimationFrame(recordFrame);
      }
      requestAnimationFrame(recordFrame);
    }));
    const cpuProfileSummary = responsivenessSession
      ? summarizeCpuProfile((await responsivenessSession.send("Profiler.stop")).profile)
      : [];
    if (responsivenessSession) {
      await responsivenessSession.send("Profiler.disable");
      await responsivenessSession.detach();
    }
    const frameTimings = await page.evaluate(() => {
      const samples = window.__droneSimFrameTimings || [];
      const summarize = (key) => {
        const values = samples.map((sample) => sample[key]).sort((a, b) => a - b);
        const percentile = (fraction) => values.length
          ? values[Math.ceil(values.length * fraction) - 1]
          : 0;
        return {
          p50: percentile(0.50),
          p95: percentile(0.95),
          maximum: values.length ? values[values.length - 1] : 0,
        };
      };
      return {
        sampleCount: samples.length,
        input: summarize("input"),
        simulation: summarize("simulation"),
        render: summarize("render"),
        bufferSwap: summarize("bufferSwap"),
        total: summarize("total"),
      };
    });
    if (responsiveness.frameCount < 30
        || responsiveness.maximumFrameGapMilliseconds > 1500)
    {
      throw new Error(
        `첫 화면 뒤 갱신이 멈췄습니다. ${responsiveness.frameCount}프레임, `
        + `최대 공백 ${Math.round(responsiveness.maximumFrameGapMilliseconds)}ms, `
        + `브라우저 긴 작업 ${JSON.stringify(responsiveness.longestTasks)}, `
        + `CPU 표본 상위 함수 ${JSON.stringify(cpuProfileSummary)}, `
        + `시뮬레이터 프레임 구간 ${JSON.stringify(frameTimings)}`);
    }
    console.log(
      `Chromium에서 첫 화면, 비행 조작, 상태 갱신을 확인했습니다. `
      + `${responsiveness.frameCount}프레임, 최대 공백 `
      + `${Math.round(responsiveness.maximumFrameGapMilliseconds)}ms, `
      + `시뮬레이터 프레임 구간 ${JSON.stringify(frameTimings)}`);

    await page.reload({ waitUntil: "domcontentloaded" });
    await page.waitForFunction(
      () => document.querySelector("#startup-status")?.hidden === true,
      null,
      { timeout: 45000 }
    );
    await page.waitForFunction(
      () => document.querySelector("#web-gcs-mode")?.textContent === "grounded",
      null,
      { timeout: 10000 }
    );
    await page.waitForTimeout(100);
    if (webGlErrors.length > 0)
      throw new Error(`화면을 다시 띄우는 동안 WebGL 오류가 발생했습니다: ${webGlErrors.join(" | ")}`);
    await page.waitForFunction(
      () => document.querySelector("#inspection-target")?.options.length > 1,
      null,
      { timeout: 10000 }
    );
    const inspectionTarget = page.locator("#inspection-target");
    await inspectionTarget.selectOption("1");
    await page.getByRole("button", { name: "정찰 시작" }).click();
    await page.waitForFunction(
      () => {
        const status = document.querySelector("#web-gcs-status")?.textContent || "";
        return status.includes("촬영 위치로 이동 중")
          || status.includes("카메라 방향을 맞추는 중");
      },
      null,
      { timeout: 30000 }
    );
    for (const label of ["이륙", "정지", "전진", "후진", "좌측", "우측",
      "상승", "하강", "좌회전", "우회전", "착륙"]) {
      if (!await page.getByRole("button", { name: label, exact: true }).isDisabled())
        throw new Error(`정찰 임무 중 ${label} 수동 조작이 활성화되어 있습니다.`);
    }
    if (!await page.locator("#inspection-target").isDisabled()
        || await page.getByRole("button", { name: "정찰 중단" }).isDisabled())
      throw new Error("정찰 임무 중 목표 선택 또는 임무 중단 상태가 올바르지 않습니다.");
    await page.getByRole("button", { name: "일시 정지" }).click();
    await page.waitForFunction(
      () => document.querySelector("#web-gcs-mission")?.textContent
        .includes("목표 1/1: 일시 정지"),
      null,
      { timeout: 5000 }
    );
    await page.getByRole("button", { name: "재개" }).click();
    await page.waitForFunction(
      () => {
        const status = document.querySelector("#web-gcs-status")?.textContent || "";
        return status.includes("촬영 위치로 이동 중")
          || status.includes("카메라 방향을 맞추는 중");
      },
      null,
      { timeout: 5000 }
    );
    try {
      await page.waitForFunction(
        () => {
          const status = document.querySelector("#web-gcs-mission")?.textContent || "";
          return status.includes("정찰 완료")
            || status.includes("실패:");
        },
        null,
        { timeout: 240000 }
      );
    } catch {
      throw new Error(`정찰 임무가 끝나지 않았습니다: ${await telemetry.innerText()}`);
    }
    const missionStatus = await telemetry.innerText();
    if (!missionStatus.includes("정찰 완료 (1개 위치)"))
      throw new Error(`정찰 임무가 끝나지 않았습니다: ${missionStatus}`);
    for (const label of ["이륙", "정지", "전진", "후진", "좌측", "우측",
      "상승", "하강", "좌회전", "우회전", "착륙"]) {
      if (await page.getByRole("button", { name: label, exact: true }).isDisabled())
        throw new Error(`정찰 임무 종료 뒤 ${label} 수동 조작이 다시 활성화되지 않았습니다.`);
    }
    if (await page.locator("#inspection-target").isDisabled()
        || await page.getByRole("button", { name: "정찰 시작" }).isDisabled()
        || !await page.getByRole("button", { name: "정찰 중단" }).isDisabled())
      throw new Error("정찰 임무 완료 뒤 조작 상태가 올바르게 복구되지 않았습니다.");
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
