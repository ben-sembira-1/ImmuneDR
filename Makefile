.DEFAULT_GOAL := default_make

build_sitl_docker:
	docker build -t sitl --file=docker/Dockerfile .

run_tests_in_docker:
	docker run -it --detach --rm --name=sitl_container -v $(pwd):/home/pilot/app sitl
	docker exec -it sitl_container /bin/bash /home/pilot/screen_record/start_and_record_virtual_Xserver.sh; docker container kill --signal=SIGTERM sitl_container

default_make:
	echo "Please run a command."
