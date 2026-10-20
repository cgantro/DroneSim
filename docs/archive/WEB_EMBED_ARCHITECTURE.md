# Shared-origin browser embed design

The repository Pages addresses `https://cgantro.github.io/GraspLink/` and `https://cgantro.github.io/Portfolio/` share **one origin** (scheme + host + port). The Portfolio should **reference a module served from GraspLink Pages**, not copy a stale `index.html` into its own repository.

## Artifacts and ownership

- `index.html` remains the compatibility-oriented standalone simulator until the Portfolio migration has been validated.
- With `emscripten-web`, CMake also builds `grasplink-embed.js` using `-pthread`, `-sMODULARIZE=1`, `-sEXPORT_NAME=createGraspLink`, and embedded virtual filesystem assets. The Pages workflow publishes it under `/GraspLink/embed/`.
- `/GraspLink/embed/manifest.json` exposes the deployed Git commit SHA and module URL. The Portfolio can fetch it with cache revalidation and append the SHA as a script query to avoid stale code.
- A browser application provides the canvas to `createGraspLink({ canvas: ... })` rather than loading an HTML document in an iframe.

## Requirements before replacing the Portfolio iframe

1. Portfolio top-level document must be **cross-origin isolated**. Its own `/Portfolio/`-scoped COI Service Worker must control a reloaded navigation, and all external CSS/fonts/assets must pass COEP / CORS / CORP requirements.
2. Check `window.crossOriginIsolated`, `typeof SharedArrayBuffer === "function"`, and the presence of initialized pthread workers. Loading a JS module does not establish true worker execution.
3. The generated Emscripten classic JS module must be executed via a normal script loader (not treated as ESM); pass the canvas to the factory.
4. A React SPA needs safe module teardown: browser event callbacks, the Emscripten main loop, OpenGL context, app-owned native objects and worker pool must not outlive an unmounted canvas. Until teardown is verified, a full document navigation on leaving the simulator is safer than remounting the module repeatedly.
5. Keep the standalone simulator link as fallback. Do not silently downgrade to a single-thread build when the user asked for Pthreads.
6. Exercise the published module in a cross-origin-isolated **Portfolio host page**, not only a successful CMake build. Both routes are same-origin but their Service Worker scopes differ.

The current Portfolio has code to **unregister** an older COI Service Worker. That migration code must be removed/replaced as part of the embed migration.

**Build status:** the reusable bundle and CI publication are implemented here as an additive, unverified deployment contract. A live Portfolio Canvas integration, native teardown API, and browser smoke test are follow-up requirements; do not describe them as finished until proven.
