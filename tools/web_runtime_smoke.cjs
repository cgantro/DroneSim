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
    const page = await browser.newPage();
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
      `Chromium에서 첫 화면과 이후 갱신을 확인했습니다. `
      + `${responsiveness.frameCount}프레임, 최대 공백 `
      + `${Math.round(responsiveness.maximumFrameGapMilliseconds)}ms`);
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
