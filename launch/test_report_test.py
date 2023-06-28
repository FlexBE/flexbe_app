# Copyright 2023 Philipp Schillinger, Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Philipp Schillinger,
#      Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import os
import sys
import pytest

import launch
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory


@pytest.mark.rostest
def generate_launch_description():
    flexbe_app_dir = get_package_share_directory('flexbe_app')
    TEST_PROC_PATH = os.path.join(flexbe_app_dir, 'test_report')

    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    test_report = launch.actions.ExecuteProcess(
        cmd=[sys.executable, TEST_PROC_PATH],
        env=proc_env,
        output='screen',
        sigterm_timeout=launch.substitutions.LaunchConfiguration('sigterm_timeout', default=90),
        sigkill_timeout=launch.substitutions.LaunchConfiguration('sigkill_timeout', default=90)
    )

    return (
        launch.LaunchDescription([
            test_report,
            launch_testing.actions.ReadyToTest()
        ]),
        {
            'test_report': test_report,
        }
    )
