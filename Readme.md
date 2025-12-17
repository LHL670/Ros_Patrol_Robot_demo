# ROS 2 AI 智慧巡邏機器人模擬 (Turtlesim 版)

## 1. 專案概述 (Overview)

本專案是一個進階的 **ROS 2 Humble** 多機器人互動範例，模擬了一場具備 AI 策略的「警匪追逐戰」。

不再是單純的巡邏，本專案實作了 **Multi-Agent System (多代理系統)**。巡邏者與入侵者各自擁有獨立的決策邏輯，包含預判攔截、據點導航、恐慌逃跑模式，以及基於 **PPO (Proximal Policy Optimization)** 架構的避障推論。

### 角色

* **巡邏者 (Police)**:
  * **據點巡邏**: 在搜索模式下，依序前往隨機生成的巡邏點，不再盲目亂轉。
  * **預判攔截**: 追捕時會計算入侵者速度向量，直接前往「未來位置」進行攔截。
  * **視覺化**: 搜索時軌跡為**綠色**，發現目標後變為**青色**。

* **入侵者 (Intruder)**:
  * **PPO 架構**: 採用強化學習的推論架構，結合勢場法 (Potential Fields) 進行決策。
  * **據點逃脫**: 生成後會獲得隨機逃跑路線 (Waypoints)，試圖依序闖關。
  * **恐慌模式 (Panic Mode)**: 當巡邏者靠近 (< 3.5m) 時，強制切斷導航，啟動**黃色警報**，全力反向逃跑與牆角彈射。
  * **智慧生成**: 確保生成位置遠離巡邏者，杜絕「落地成盒」。

* 利用動態軌跡顏色 (Trail Color) 即時反映角色狀態。

---

## 2. 系統架構 (System Architecture)

### 2.1 檔案結構

```text
ros2_patrol_project/
├── docker-compose.yaml      # 容器啟動設定 (包含 GUI 顯示掛載)
├── Dockerfile               # 環境建置檔 (ROS 2 Humble + Python 依賴)
├── requirements.txt         # Python 依賴清單 (NumPy 等)
├── turtlesim_chase.py       # 警匪追逐主程式 
└── README.md                # 專案說明文件
```

### 2.2 角色狀態機 (State Machines)

系統同時運行兩套狀態邏輯：

| 角色 | 狀態 (State) | 行為描述 | 視覺回饋 |
| :--- | :--- | :--- | :--- |
| **Police** | **SEARCH** | 沿著隨機生成的巡邏點 (Waypoints) 移動搜索。 | 🟢 綠色軌跡 |
| | **CHASING** | 發現入侵者 (FOV內)，加速並預判攔截。 | 🔵 青色軌跡 |
| | **ARRESTED** | 距離 < 0.8m，逮捕成功，凍結現場。 | 🛑 停止 |
| **Intruder** | **NAVIGATE** | 安全狀態，依序前往隨機逃跑據點。 | 🔴 紅色軌跡 |
| | **PANIC** | 危險狀態 (< 3.5m)，忽視據點，全力避障與反向逃跑。 | 🟡 黃色粗軌跡 |

### 2.3 演算法邏輯

* **視線模擬 (FOV)**: 巡邏者僅能看見前方 90 度扇形、距離 5 公尺內的目標。
* **角落彈射 (Corner Ejection)**: 當入侵者被逼入死角時，產生強力向心向量，強制脫困。
* **PPO 獎勵函數 (Reward Function)**:
    * $R = R_{survival} + R_{distance} - P_{wall} - P_{corner} - P_{caught}$
    * 即時計算並顯示於終端機，評估逃跑決策品質。

---

## 3. 環境需求 (Prerequisites)

* **作業系統**: Linux (Ubuntu 建議) 或 Windows (WSL2)。
* **軟體**:
    * Docker Engine & Docker Compose
    * X11 Server (用於顯示 Turtlesim GUI)。

---

## 4. 執行步驟 (Execution Steps)

### 步驟 1：建立專案
確保資料夾內包含 `Dockerfile`, `docker-compose.yaml`, `turtlesim_chase.py` (v16) 與此 `README.md`。

### 步驟 2：開放 GUI 顯示權限
在本機終端機執行：
```bash
xhost +local:root
```

*(若顯示 "access control disabled" 即代表成功)*

### 步驟 3：啟動環境

使用 Docker Compose 建置並啟動容器：

```bash
# 建置並在背景啟動
docker compose up -d --build
```

### 步驟 4：啟動模擬器 (Terminal 1)

開啟一個新的終端機，進入容器並執行 Turtlesim：

```bash
# 進入容器
docker exec -it patrol_bot_container bash

# 啟動模擬器 (請保持此視窗開啟)
ros2 run turtlesim turtlesim_node
```

### 步驟 5：啟動巡邏程式 (Terminal 2)

開啟**另一個**新的終端機，進入同一個容器執行控制程式：

```bash
# 進入容器
docker exec -it patrol_bot_container bash

# 執行巡邏機器人
python3 turtlesim_chase_fov.py
```

### 預期體驗

1. **回合開始**: 入侵者 (Intruder) 在遠處生成，開始跑向它的第一個據點 (紅色軌跡)。

2. **巡邏**: 巡邏者 (Turtle1) 沿著綠色軌跡巡邏。

3. **發現**: 當入侵者進入巡邏者視野，巡邏者軌跡變藍，加速追捕。

4. **恐慌**: 當兩者靠近，入侵者軌跡變**黃色 (粗線)**，開始瘋狂逃竄。

5. **逮捕**: 追捕成功後，顯示結算獎勵，3秒後重置場景，生成新的隨機據點與路線。

## 5. 參數調整 (Tuning)

可以在 `turtlesim_chase_fov.py` 的 `__init__` 區塊調整遊戲難度：

* `self.max_speed`: 巡邏者最大速度 (預設 3.5)。

* `self.intruder_speed`: 入侵者速度 (預設 2.5)。

* `self.fov_range`: 巡邏者視線距離 (預設 5.0)。

---
<!-- 
## 5. 常見問題排除 (Troubleshooting)

**Q1: 執行 `python3 turtlesim_patrol.py` 時出現 `ModuleNotFoundError: No module named 'rclpy'`？**

* **原因**: 未載入 ROS 2 環境變數。
* **解法**: 請確認您的 `Dockerfile` 內有 `source /opt/ros/humble/setup.bash` 設定，或者在容器內手動執行 `source /opt/ros/humble/setup.bash`。

**Q2: Turtlesim視窗沒有跳出來，或出現 `qt.qpa.xcb: could not connect to display`？**

* **原因**: Docker 無法連接本機的顯示伺服器。
* **解法**:
  1. 確認已執行 `xhost +local:root`。
  2. 確認 `docker-compose.yaml` 中有設定 `- /tmp/.X11-unix:/tmp/.X11-unix` 與環境變數 `DISPLAY`。
  3. 若使用 WSL2，`DISPLAY` 設定可能較為複雜，建議將 `DISPLAY` 設為 `:0` 或使用 WSLg。

**Q3: 如何停止專案？**

* 在本機終端機執行：`docker compose down`。 -->