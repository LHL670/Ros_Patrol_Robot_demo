# 使用官方 ROS 2 Humble Desktop 映像檔作為基底
# 包含 ROS 2 核心與 GUI 工具 (如 RViz, Turtlesim)
FROM osrf/ros:humble-desktop

# 設定環境變數，避免安裝過程出現互動式詢問
ENV DEBIAN_FRONTEND=noninteractive

# 1. 安裝系統依賴與 Python 工具
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    ros-humble-turtlesim \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# 2. 設定工作目錄
WORKDIR /root/workspace

# 3. 複製專案檔案進容器
# 將當前目錄下的 requirements.txt 複製進去 (若有的話)
COPY requirements.txt .

# 4. 安裝 Python 依賴
# 這裡會檢查 requirements.txt 是否存在並安裝
RUN if [ -f "requirements.txt" ]; then pip3 install -r requirements.txt; fi

# 5. 複製程式碼
COPY turtlesim_patrol.py .

# 6. 設定 ROS 環境變數 (每次進入容器自動 source)
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 預設指令：保持容器開啟，方便進入操作
CMD ["sleep", "infinity"]