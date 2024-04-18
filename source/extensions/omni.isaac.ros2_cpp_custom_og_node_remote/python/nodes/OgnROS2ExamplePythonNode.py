# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
import omni
import omni.graph.core as og
import omni.graph
import numpy


class OgnROS2ExemplePythonNode:
    @staticmethod
    def compute(db) -> bool:
        return True
