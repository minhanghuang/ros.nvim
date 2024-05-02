import argparse
import os
import re
from enum import Enum
from typing import List, Dict
import xml.etree.ElementTree as ET


class RosDistro(Enum):
    """ ros version """
    UNKNOWN = 0
    # Ros
    # https://wiki.ros.org/Distributions
    KINETIC = 110   # 2016
    LUNAR = 111     # 2017
    MELODIC = 112   # 2018
    NOETIC = 123    # 2020

    # Ros2
    # https://docs.ros.org/en/foxy/Releases.html
    DASHING = 210   # 2019
    ELOQUENT = 211  # 2019
    FOXY = 212      # 2020
    GALACTIC = 213  # 2021
    HUMBLE = 214    # 2022
    IRON = 215      # 2023


class BuildTool(Enum):
    """ build tools """
    UNKNOWN = 0
    CMAKE = 1         # mkdir -p build && cd build && cmake .. && make

    # Ros
    CATKINMAKE = 2    # catkin_make build
    CATKINTOOLS = 3   # catkin build

    # Ros2
    COLCON = 4        # colcon build


class Param:
    def __init__(self) -> None:
        self._parser = argparse.ArgumentParser(description='')
        self._start()
        self._args = self._parser.parse_args()
        return None

    def get_work_space_dir(self) -> str:
        return self._args.work_space_dir

    def get_type(self) -> str:
        return self._args.type

    def _start(self) -> None:
        self._parser.add_argument(
            '-ws',
            '--work_space_dir',
            type=str,
            help='工作区路径'
        )
        self._parser.add_argument(
            '-t',
            '--type',
            type=str,
            help='执行类型'
        )

        return None


param = Param()


class Util:

    def __init__(self, ws_dir: str = None):
        # workspace dir
        self._ws_dir: str = ws_dir

        # load ros env
        self._ros_distro: RosDistro = RosDistro.UNKNOWN

        # workspace build tool type
        self._ws_build_tool: BuildTool = BuildTool.UNKNOWN

        # all package
        self._packages: Dict[str, str] = {}

        # select build package list
        self._select_packages: Dict[str, str] = {}

        self._install_space: str = None

        self._devel_space: str = None

        self._source_space: str = None

        self._init()

    def _init(self) -> None:
        # workspace dir
        if None is self._ws_dir:
            self._ws_dir = param.get_work_space_dir()
            if None is self._ws_dir:
                raise

        # Ros version
        ros_distro: str = os.environ.get("ROS_DISTRO")
        if None is ros_distro:
            raise
        if ros_distro.upper() not in RosDistro.__members__:
            raise
        self._ros_distro = RosDistro[ros_distro.upper()]

        # build tool type
        if os.path.exists(os.path.join(self._ws_dir, "build/.built_by")):
            with open(os.path.join(self._ws_dir, "build/.built_by"), 'r') as file:
                file_content = file.readline().strip()
                if "catkin_make" == file_content:
                    self._ws_build_tool = BuildTool.CATKINMAKE
                elif "catkin build" == file_content:
                    self._ws_build_tool = BuildTool.CATKINTOOLS
        else:
            self._ws_build_tool = BuildTool.CMAKE

        # select package
        if BuildTool.CATKINMAKE == self._ws_build_tool:
            with open(os.path.join(self._ws_dir, "build/catkin_make.cache"), 'r') as file:
                line_num = 0
                for line in file:
                    if 0 == line_num:
                        # package name
                        package_names = re.findall(r'\b\w+\b', line)
                        for package_name in package_names:
                            self._select_packages[package_name] = ""
                    elif 1 == line_num:
                        # space
                        prefixes = re.findall(r'PREFIX=([^ ]+)', line)
                        for space in prefixes:
                            if "devel" in space:
                                self._devel_space = space
                            elif "install" in space:
                                self._install_space = space
                            elif "source" in space:
                                self._source_space = space
                    else:
                        break
                    line_num += 1

        elif BuildTool.CATKINTOOLS == self._ws_build_tool:
            # package name
            packages_dir = os.path.join(
                self._ws_dir, ".catkin_tools/profiles/default/packages/")
            if not os.path.exists(packages_dir):
                self._select_packages = {}
            else:
                pacakges_folder = os.listdir(packages_dir)
                for package_name in pacakges_folder:
                    if os.path.isdir(
                        os.path.join(
                            packages_dir,
                            package_name)) and "catkin_tools_prebuild" != package_name:
                        # not select default package "catkin_tools_prebuild"
                        self._select_packages[package_name] = ""

            # space
            tool_config = os.path.join(
                self._ws_dir, "build/.catkin_tools.yaml")
            if not os.path.exists(tool_config):
                self._devel_space = None
                self._install_space = None
                self._source_space = None
            else:
                with open(tool_config, "r") as file:
                    for line in file:
                        if ':' in line:
                            key, value = line.strip().split(':', 1)
                            if "devel_space" == key.strip():
                                self._devel_space = value.strip()
                            elif "install_space" == key.strip():
                                self._install_space = value.strip()
                            elif "source_space" == key.strip():
                                self._source_space = value.strip()
                pass

        else:
            # TODO:
            pass

        # all package
        exclude_dirs = []
        exclude_dirs.append(os.path.basename(self._devel_space))
        exclude_dirs.append(os.path.basename(self._install_space))
        exclude_dirs.append("logs")
        exclude_dirs.append("build")
        exclude_dirs.append(".catkin_tools")
        for root, dirs, files in os.walk(self._ws_dir, followlinks=True):
            dirs[:] = [d for d in dirs if d not in exclude_dirs]
            for file in files:
                file_path = os.path.join(root, file)
                if file == 'package.xml' and not os.path.islink(file_path):
                    name = self._get_package_name(file_path)
                    self._packages[name] = root
                    if name in self._select_packages.keys():
                        self._select_packages[name] = root
                elif os.path.islink(os.path.normpath(file_path)):
                    real_path = os.path.realpath(file_path)
                    if os.path.isfile(real_path) and real_path.endswith(
                            'package.xml'):
                        name = self._get_package_name(file_path)
                        self._packages[name] = root
                        if name in self._select_packages.keys():
                            self._select_packages[name] = root
        return None

    def _get_package_name(self, package_file: str) -> str:
        """
        get package name
        :param package_file: packages.xml path
        :return: package name
        """
        tree = ET.parse(package_file)
        root = tree.getroot()
        name_tag = root.find("name")
        return None if name_tag is None else name_tag.text

    def get_ros_distro(self) -> RosDistro:
        """
        get current env ros version
        :return: str
        """
        return self._ros_distro

    def get_ws_build_tool(self) -> BuildTool:
        """
        获取工作区编译工具
        :return:
        """
        return self._ws_build_tool

    def is_work_space(self) -> bool:
        """
        判断传入的参数是否是Ros工作区
        :return:
        """
        return self._ws_dir is not None

    def has_compile_commands_file(self) -> bool:
        """
        判断compile_commands.json文件是否已经村长
        :return: bool
        """
        if not self.is_work_space():
            return False
        work_space_dir = param.get_work_space_dir()
        compile_commands_file = os.path.join(
            work_space_dir, "compile_commands.json")
        if not os.path.exists(compile_commands_file):
            return False
        return True

    def get_build_package_list(self) -> List[str]:
        """
        get build package name
        :return: [path]
        """
        if not self.is_work_space():
            return False

        return [name for name, _ in self._select_packages.items()]

    def get_package_path_list(self) -> List[str]:
        """
        获取所有package绝对路径
        :return: [path]
        """
        if not self.is_work_space():
            return []

        return [name for name, _ in self._packages.items()]

    def get_package_path(self, package_name: str) -> str:
        """
        get package abs path
        :param package_name: name
        :return: path
        """
        if package_name not in self._packages.keys():
            return None
        return self._packages[package_name]


if __name__ == "__main__":
    print("workspace: {}".format(param.get_work_space_dir()))
    util = Util(param.get_work_space_dir())
    print("all package: {}".format(util.get_package_path_list()))
    print("build package: {}".format(util.get_build_package_list()))
    print(
        "package path: {}".format(
            util.get_package_path(
                util.get_build_package_list()[0])))
