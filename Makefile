IMAGE_NAME = ros-noetic-zsh:latest
CONTAINER_NAME = ros-noetic-zsh

all: build run

build:
	docker build -t $(IMAGE_NAME) .
	
run:
	xhost +local:root

	docker run -it --rm \
		--privileged \
		-e DISPLAY=$$DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(HOME)/.Xauthority:/root/.Xauthority:rw \
		-e XAUTHORITY=/root/.Xauthority \
		-e XDG_RUNTIME_DIR=/tmp \
		-e QT_X11_NO_MITSHM=1 \
		--net=host \
		--name $(CONTAINER_NAME) \
		--ulimit nofile=1024:524288 \
		--mount type=bind,source=$(shell pwd)/catkin_ws,target=/root/catkin_ws \
		$(IMAGE_NAME) /bin/zsh

	xhost -local:root


stop:
	-docker stop $(CONTAINER_NAME)
	-docker rm $(CONTAINER_NAME)

clean: stop
	-docker rmi $(IMAGE_NAME)

attach:
	-docker exec -it $(CONTAINER_NAME) /bin/zsh

rviz: ## 直接在容器裡跑 rviz
	-docker exec -it $(CONTAINER_NAME) rviz
