# encoding=utf-8
from __future__ import print_function
import sys
import textwrap

import setuptools

PYTHON_VERSION = sys.version_info[:2]
if PYTHON_VERSION != (2, 7) and PYTHON_VERSION < (3, 5):
    print("This roarm_sdk version requires Python 2.7, 3.5 or later.")
    sys.exit(1)

# Keep build metadata independent from importing roarm_sdk itself. Importing the
# package imports the serial transport, but build isolation resolves runtime
# dependencies only after metadata generation. Importing the package here therefore
# made a normal `pip install` fail before pyserial could be installed.
VERSION = "0.1.1"
AUTHOR = "waveshareteam"
AUTHOR_EMAIL = "2849678712@qq.com"
GIT_URL = "https://github.com/waveshareteam/waveshare_roarm_sdk.git"

install_requires = [
    "pyserial",
    "requests",
]

if sys.version_info >= (3, 10):
    install_requires.append("simplejson")

try:
    long_description = (
        open("README.md", encoding="utf-8").read()
        + open("doc/README.md", encoding="utf-8").read()
    )
except (FileNotFoundError, IOError):
    long_description = textwrap.dedent(
        """
        waveshare roarm sdk
        """
    )

setuptools.setup(
    name="roarm_sdk",
    version=VERSION,
    author=AUTHOR,
    author_email=AUTHOR_EMAIL,
    description="waveshare roarm sdk.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url=GIT_URL,
    packages=setuptools.find_packages(),
    include_package_data=True,
    package_data={"roarm_sdk": ["*.json"]},
    classifiers=[
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "License :: OSI Approved :: GNU Affero General Public License v3",
        "Operating System :: OS Independent",
    ],
    install_requires=install_requires,
    python_requires=">=2.7, !=3.0.*, !=3.1.*, !=3.2.*, !=3.3.*, !=3.4.*",
)
