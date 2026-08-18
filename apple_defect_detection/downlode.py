import yt_dlp

def download_youtube_video(url):
    # 다운로드 옵션 설정
    ydl_opts = {
        'format': 'best',
        'outtmpl': '%(title)s.%(ext)s', 
    }

    print(f"다운로드를 시작합니다: {url}")
    
    try:
        with yt_dlp.YoutubeDL(ydl_opts) as ydl:
            ydl.download([url])
        print("다운로드가 완료되었습니다!")
    except Exception as e:
        print(f"다운로드 중 오류가 발생했습니다: {e}")

# 다운로드할 유튜브 영상 링크를 아래에 입력하세요
video_link = "https://www.youtube.com/shorts/r9K9fF6UHuE"

download_youtube_video(video_link)