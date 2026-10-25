const path = require("node:path");
const { spawn } = require("node:child_process");
const { chromium } = require(path.join(
  process.env.RUNNER_TEMP,
  "web-smoke",
  "node_modules",
  "playwright"
));

const port = 4173;
const server = spawn("python3", [
  "-m",
  "http.server",
  String(port),
  "--bind",
  "127.0.0.1",
  "--directory",
  path.resolve("_site"),
], { stdio: "ignore" });

const diagnostics = [];
let lastStartupStage = "브라우저 페이지를 여는 중";

function record(message) {
  diagnostics.push(message);
  console.log(message);
}

async function waitForServer() {
  for (let attempt = 0; attempt < 50; ++attempt) {
    try {
      const response = await fetch(`http://127.0.0.1:${port}/`);
      if (response.ok)
        return;
    } catch {
    }
    await new Promise((resolve) => setTimeout(resolve, 100));
  }
  throw new Error("시험용 웹 서버가 시작되지 않았습니다.");
}

async function run() {
  let browser;
  try {
    await waitForServer();
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
      const text = message.text();
      if (text.startsWith("[시뮬레이터 시작] "))
        lastStartupStage = text.slice("[시뮬레이터 시작] ".length);
      if (message.type() === "error" || text.startsWith("[시뮬레이터 시작] "))
        record(`브라우저 ${message.type()}: ${text}`);
    });
    page.on("pageerror", (error) => record(`페이지 오류: ${error.stack || error.message}`));
    page.on("requestfailed", (request) => {
      record(`요청 실패: ${request.url()} (${request.failure()?.errorText || "원인 없음"})`);
    });
    page.on("response", (response) => {
      if (response.status() >= 400)
        record(`HTTP ${response.status()}: ${response.url()}`);
    });

    await page.goto(`http://127.0.0.1:${port}/`, {
      waitUntil: "domcontentloaded",
      timeout: 15000,
    });
    await page.waitForFunction(
      () => document.querySelector("#startup-status")?.hidden === true,
      null,
      { timeout: 45000 }
    );
    record("브라우저에서 첫 화면 표시를 확인했습니다.");
  } catch (error) {
    record(`브라우저 검사 실패: ${error.stack || error.message}`);
    record(`마지막 시작 단계: ${lastStartupStage}`);
    if (diagnostics.length === 0)
      record("브라우저 오류·실패한 요청·C++ 시작 단계가 기록되지 않았습니다.");
    process.exitCode = 1;
  } finally {
    if (browser)
      await browser.close().catch((error) => record(`브라우저 종료 오류: ${error.message}`));
    server.kill();
  }
}

run();
