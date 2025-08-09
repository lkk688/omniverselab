# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
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

"""Perform validation and configuration of the Jupyter Notebook environment for the project."""

import logging
import logging.config

log = logging.getLogger(__name__)


CONTENT_DIRECTORY = "./content"
"""Relative path to the directory where the USD content is located."""


def get_content_directory() -> str:
    """
    Return the relative path to the directory where the USD content is located.

    Parameters:
        None

    Return:
        str: The relative path to the directory where the USD content is located.
    
    """
    return CONTENT_DIRECTORY


def get_content_output_directory() -> str:
    """
    Return the relative path to the directory where the glTF assets converted from the USD content are located.

    Parameters:
        None

    Return:
        str: The relative path to the directory where the glTF assets converted from the USD content are located.

    """
    return f"{CONTENT_DIRECTORY}/output"
