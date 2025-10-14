# Repository Guidelines

## Project Structure & Module Organization
- Solution: `Cloud2Pano.sln` (Visual Studio C++), located at the repo root.
- App project: `Cloud2Pano/`; sources in `Cloud2Pano/source/` (e.g., `MeshLoader.cpp`, `Camera.h`).
- Shaders: `Cloud2Pano/shader/` (e.g., `shader.vert`, `shader.frag`).
- Build outputs: `x64/Debug/` and `x64/Release/`; executable: `x64/Debug/Cloud2Pano.exe`.
- Third‑party (vcpkg via VS): GLFW, GLEW, GLM, tinyobjloader. DLLs deploy next to the exe.

## Build, Test, and Development Commands
- Build (Debug x64): `msbuild Cloud2Pano.sln /t:Build /p:Configuration=Debug;Platform=x64`
- Build (Release x64): `msbuild Cloud2Pano.sln /t:Build /p:Configuration=Release;Platform=x64`
- Run (Debug): `./x64/Debug/Cloud2Pano.exe`
- Visual Studio: open `Cloud2Pano.sln`, select `x64` + `Debug/Release`, then Build.

## Coding Style & Naming Conventions
- Language: C++17 (uses `std::filesystem`).
- Indentation: prefer 4 spaces; match existing per‑file style (some headers use tabs).
- Braces: K&R style (`void foo() { ... }`).
- Naming: types `PascalCase` (e.g., `GLMesh`); functions `camelCase` (e.g., `loadObjFolderBuildGL`); locals `camelCase`; macros/consts `ALL_CAPS` or `kPascalCase`.
- Headers: use `#pragma once`. Group includes: project first, then system/3rd‑party.

## Testing Guidelines
- No formal suite yet. Place new tests under `Cloud2Pano/tests/` and name `*.tests.cpp`.
- Aim for fast, focused unit tests (mesh loading, OBJ parsing). Target ≥80% coverage for new code.
- Keep tests deterministic; avoid GPU‑dependent assertions when possible.

## Commit & Pull Request Guidelines
- Commits: imperative mood, concise scope. Example: `MeshLoader: fix index buffer overflow`.
- Reference issues: `Fixes #123` or `Refs #123`.
- PRs: clear description, reproduction steps, before/after screenshots for rendering changes, risk notes, and linked issues.

## Security & Configuration Tips
- Build with VS x64 toolset (v143). Ensure runtime DLLs (`glfw3.dll`, `glew32(d).dll`) sit next to the exe; Release uses non‑`d` DLLs.
- Requires an OpenGL‑capable GPU/driver. If startup fails, update GPU drivers and verify DLL presence.

## Agent Notes
- Do not rename public APIs or move files without updating the `.vcxproj` and filters.
- Keep changes minimal and focused; match existing patterns in `Cloud2Pano/source/` and `Cloud2Pano/shader/`.

