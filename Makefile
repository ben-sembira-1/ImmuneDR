.DEFAULT_GOAL := default_make

build_sitl_docker:
	docker build --rm -t sitl --file=docker/Dockerfile .

default_make:
	echo "Please run a command."
