IMAGE_NAME = ros-noetic-zsh:latest
CONTAINER_NAME = ros-noetic-zsh

all: build run

build:
	docker build -t $(IMAGE_NAME) .

run:
	# xhost +local:root  # 開放 X11 給 root 用戶（為了顯示 rviz，用不到可以註解）

	mkdir -p $(HOME)/.claude-docker

	docker run -it --rm \
		--privileged \
		--net=host \
		--name $(CONTAINER_NAME) \
		--ulimit nofile=1024:524288 \
		--mount type=bind,source=$(shell pwd)/catkin_ws,target=/root/catkin_ws \
		--mount type=bind,source=$(shell pwd)/agent_ref,target=/root/agent_ref \
		--mount type=bind,source=$(HOME)/.claude-docker,target=/root/.claude-persist \
		-e CLAUDE_CONFIG_DIR=/root/.claude-persist \
		$(IMAGE_NAME) /bin/zsh

	# 登入記憶化：Claude 的設定目錄 (含 .credentials.json 登入憑證) 指到 /root/.claude-persist，
	# 並掛到主機 $(HOME)/.claude-docker。第一次進容器跑 `claude` 登入一次，token 就會留在主機，
	# 之後即使容器 --rm 重建也不必再登入。rosdebug skill 由 entrypoint 複製進同一目錄。

	# 下方是顯示畫面所需的設定，已註解不用
	# -e DISPLAY=$$DISPLAY                              # 顯示畫面所需 (rviz)
	# -v /tmp/.X11-unix:/tmp/.X11-unix:rw               # 顯示畫面所需 (X11 socket)
	# -v $(HOME)/.Xauthority:/root/.Xauthority:rw       # 顯示畫面所需 (權限)
	# -e XAUTHORITY=/root/.Xauthority                   # 顯示畫面所需 (X11驗證)
	# -e XDG_RUNTIME_DIR=/tmp                           # 顯示畫面所需 (runtime dir)
	# -e QT_X11_NO_MITSHM=1                             # 顯示畫面所需 (Qt修正)

	# xhost -local:root  # 關閉 X11 root 訪問（顯示畫面結束後用）

stop:
	-docker stop $(CONTAINER_NAME)
	-docker rm $(CONTAINER_NAME)

clean: stop
	-docker rmi $(IMAGE_NAME)

attach:
	-docker exec -it $(CONTAINER_NAME) /bin/zsh

rviz: ## 直接在容器裡跑 rviz
	-docker exec -it $(CONTAINER_NAME) rviz
