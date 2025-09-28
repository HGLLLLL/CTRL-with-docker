IMAGE_NAME = ros-noetic-zsh:latest
CONTAINER_NAME = ros-noetic-zsh

all: build run

build:
	docker build -t $(IMAGE_NAME) .
	
run:
	docker run -it --rm \
	    --privileged \
	    -e XDG_RUNTIME_DIR=/tmp \
	    --net=host \
	    --name $(CONTAINER_NAME) \
	    --ulimit nofile=1024:524288 \
	    --mount type=bind,source=$(shell pwd)/catkin_ws,target=/root/catkin_ws \
	    $(IMAGE_NAME) /bin/zsh


stop:
	-docker stop $(CONTAINER_NAME)
	-docker rm $(CONTAINER_NAME)

clean: stop
	-docker rmi $(IMAGE_NAME)

attach:
	-docker exec -it $(CONTAINER_NAME) /bin/zsh

rviz: ## 直接在容器裡跑 rviz
	-docker exec -it $(CONTAINER_NAME) rviz
