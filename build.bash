#!/usr/bin/env bash

#
# Copyright (C) 2018 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# Builds the bluerov_ws Docker image.
# Usage: ./build.bash
image_name=bluerov_ws
image_tag=humble

if [ ! -f "docker/Dockerfile" ]; then
    echo "Err: docker/Dockerfile not found. Run from the workspace root."
    exit 1
fi

image_plus_tag=$image_name:$(export LC_ALL=C; date +%Y_%m_%d_%H%M)
docker build --rm -t $image_plus_tag -f docker/Dockerfile docker && \
docker tag $image_plus_tag $image_name:$image_tag && \
echo "Built $image_plus_tag and tagged as $image_name:$image_tag"
echo "To run:"
echo "./run.bash $image_name:$image_tag"
