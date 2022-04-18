


default_make:
	echo "Please run a command."

build_sitl_docker:
	docker build -t sitl --file=docker/Dockerfile .

run_tests_in_docker:
	docker run -it --rm -v $(shell pwd):/home/pilot/app sitl python3 -m pytest . --pdb
