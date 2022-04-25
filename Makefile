PYTEST_ARGS ?= ""
MAVPROXY ?=
ifdef MAVPROXY
MAVPROXY_PORT ?= 5594
MAVPROXY_PORT_MAPPING := -p ${EXTERNAL_GCS_PORT}:${MAVPROXY_PORT}
endif
EXTERNAL_GCS_PORT ?= 5763

default_make:
	echo "Please run a command."

tests_shell:
	docker run -it --rm -v $(shell pwd):/home/pilot/app -p 5763:${EXTERNAL_GCS_PORT} sitl bash

build_sitl_docker:
	docker build -t sitl --file=docker/Dockerfile .

run_tests_in_docker:
ifdef MAVPROXY
	mavproxy.py --master tcp:localhost:${EXTERNAL_GCS_PORT} --non-interactive --map --console > /tmp/mavproxy.output 2> /tmp/mavproxy.err &
endif
	docker run -it --rm -e MAVPROXY_PORT=${MAVPROXY_PORT} -v $(shell pwd):/home/pilot/app ${MAVPROXY_PORT_MAPPING} sitl python3 -m pytest . ${PYTEST_ARGS}
