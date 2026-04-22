@echo off
setlocal

cd /d "%~dp0"
echo Updating repository in: %cd%

git rev-parse --is-inside-work-tree >nul 2>&1
if errorlevel 1 (
  echo ERROR: This folder is not a Git repository.
  pause
  exit /b 1
)

echo.
echo Fetching latest from origin...
git fetch origin
if errorlevel 1 (
  echo ERROR: git fetch failed.
  pause
  exit /b 1
)

echo.
echo Resetting local main to origin/main...
git checkout main >nul 2>&1
git reset --hard origin/main
if errorlevel 1 (
  echo ERROR: hard reset failed.
  pause
  exit /b 1
)

echo.
echo Cleaning untracked files/folders...
git clean -fd
if errorlevel 1 (
  echo ERROR: git clean failed.
  pause
  exit /b 1
)

echo.
echo Update complete.
git log --oneline -n 1
echo.
pause
