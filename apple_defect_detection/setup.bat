@echo off
chcp 65001 > nul
echo ============================================
echo   Apple 프로젝트 환경 자동 설치 시작
echo ============================================
echo.

REM ===== Python 설치 여부 확인 =====
python --version > nul 2>&1
if errorlevel 1 (
    echo [오류] Python이 설치되어 있지 않습니다.
    echo https://www.python.org 에서 Python 3.11 이상을 먼저 설치해주세요.
    pause
    exit /b 1
)

REM ===== 가상환경 생성 =====
if exist apple_env (
    echo 기존 가상환경^(apple_env^)이 이미 존재합니다. 재사용합니다.
) else (
    echo 가상환경 생성 중...
    python -m venv apple_env
)

REM ===== 가상환경 활성화 =====
call apple_env\Scripts\activate.bat

REM ===== pip 업그레이드 =====
echo pip 업그레이드 중...
python -m pip install --upgrade pip -q

REM ===== NVIDIA GPU 감지 =====
echo GPU 확인 중...
nvidia-smi > nul 2>&1
if errorlevel 1 (
    echo GPU를 찾을 수 없습니다. CPU 버전으로 설치합니다.
    set TORCH_CMD=pip install torch torchvision
) else (
    echo NVIDIA GPU 감지됨! CUDA 버전으로 설치합니다.
    set TORCH_CMD=pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
)

REM ===== PyTorch 설치 =====
echo PyTorch 설치 중... ^(시간이 걸릴 수 있습니다^)
%TORCH_CMD%

REM ===== 나머지 패키지 설치 =====
echo 나머지 패키지 설치 중...
pip install -r requirements.txt

echo.
echo ============================================
echo   설치 완료! GPU 인식 확인 중...
echo ============================================
python -c "import torch; print('CUDA 사용 가능:', torch.cuda.is_available()); print('GPU 이름:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else '없음 (CPU 모드)')"

echo.
echo 모든 설치가 완료되었습니다!
echo 앞으로는 아래 명령어로 가상환경을 켜고 실행하시면 됩니다:
echo   apple_env\Scripts\activate.bat
echo   python apple_video.py
echo.
pause