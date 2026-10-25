const fs = require("node:fs");
const http = require("node:http");
const path = require("node:path");
const { chromium } = require(path.join(
  process.env.RUNNER_TEMP,
  "web-smoke",
  "node_modules",
  "playwright"
));

const port = 4173;
const siteRoot = path.resolve("_site");
const mimeTypes = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".wasm": "application/wasm",
};
const startupStages = [];
let finishStartup;
const startupFinished = new Promise((resolve) => {
  finishStartup = resolve;
});

function log(message) {
  console.log(message);
}

function createServer() {
  return http.createServer((request, response) => {
    const requestPath = new URL(request.url, `http://127.0.0.1:${port}`).pathname;
    if (request.method === "POST" && requestPath === "/__startup_stage") {
      let stage = "";
      request.setEncoding("utf8");
      request.on("data", (chunk) => { stage += chunk; });
      request.on("end", () => {
        startupStages.push(stage);
        log(`시작 단계: ${stage}`);
        if (stage === "첫 화면 표시 완료")
          finishStartup();
        response.writeHead(204).end();
      });
      return;
    }

    const relativePath = decodeURIComponent(requestPath).replace(/^\/+/, "");
    const filePath = path.resolve(siteRoot, relativePath || "index.html");
    if (filePath !== siteRoot && !filePath.startsWith(`${siteRoot}${path.sep}`)) {
      response.writeHead(403).end();
      return;
    }

    fs.createReadStream(filePath)
      .on("error", () => response.writeHead(404).end())
      .on("open", () => {
        const contentType = mimeTypes[path.extname(filePath)] || "application/octet-stream";
        response.writeHead(200, { "Content-Type": contentType });
      })
      .pipe(response);
  });
}

async function run() {
  const server = createServer();
  let browser;
  let timedOut = false;
  try {
    await new Promise((resolve, reject) => {
      server.once("error", reject);
      server.listen(port, "127.0.0.1", resolve);
    });

    browser = await chromium.launch({
      headless: true,
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
        log(`브라우저 오류: ${message.text()}`);
    });
    page.on("pageerror", (error) => log(`페이지 오류: ${error.stack || error.message}`));
    page.on("requestfailed", (request) => {
      log(`요청 실패: ${request.url()} (${request.failure()?.errorText || "원인 없음"})`);
    });

    await page.goto(`http://127.0.0.1:${port}/`, {
      waitUntil: "commit",
      timeout: 15000,
    });

    let startupTimeout;
    timedOut = await Promise.race([
      startupFinished.then(() => false),
      new Promise((resolve) => {
        startupTimeout = setTimeout(() => resolve(true), 30000);
      }),
    ]);
    clearTimeout(startupTimeout);
    if (timedOut) {
      log(`첫 화면이 나오지 않았습니다. 마지막 시작 단계: ${startupStages.at(-1) || "시작 단계 신호가 없음"}`);
      process.exitCode = 1;
    } else {
      log("Chromium에서 첫 화면 표시를 확인했습니다.");
    }
  } catch (error) {
    log(`웹 실행 검사 오류: ${error.stack || error.message}`);
    log(`마지막 시작 단계: ${startupStages.at(-1) || "시작 단계 신호가 없음"}`);
    process.exitCode = 1;
  } finally {
    if (browser) {
      const browserProcess = browser.process();
      await Promise.race([
        browser.close(),
        new Promise((resolve) => setTimeout(resolve, 2000)),
      ]);
      browserProcess?.kill("SIGKILL");
    }
    await new Promise((resolve) => server.close(resolve));
  }
}

run();
