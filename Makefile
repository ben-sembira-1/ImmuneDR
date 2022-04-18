PYTEST_ARGS ?= ""
MAVPROXY ?=
EXTERNAL_GCS_PORT ?= 5763

default_make:
	echo "Please run a command."

build_sitl_docker:
	docker build -t sitl --file=docker/Dockerfile .

run_tests_in_docker:
ifdef MAVPROXY
	mavproxy.py --master tcp:localhost:${EXTERNAL_GCS_PORT} --force-connected --daemon --non-interactive --map --console > /tmp/mavproxy.output 2> /tmp/mavproxy.err &
endif
	docker run -it --rm -v $(shell pwd):/home/pilot/app -p 5763:${EXTERNAL_GCS_PORT} sitl python3 -m pytest . ${PYTEST_ARGS}