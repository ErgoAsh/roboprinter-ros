#!/bin/bash

docker exec -it $(docker ps --latest --quiet) /bin/bash -ci "source /root/.bashrc && /bin/bash"
