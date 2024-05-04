import argparse
import os
import re
import json
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


class Package:

    def __init__(self):
        # build package
        self.enable: bool = False

        # package name
        self.name: str = None

        # package abs path
        self.path: str = None

        # package is software type
        self.software: bool = False

        # compile_commands.json path
        self.compile_commands_path: str = None

        # depend package
        self.depend_package: List[str] = []


class WorkSpace:

    def __init__(self):
        # ros version
        self.ros_distro: RosDistro = None

        # workspace abs path
        self.path: str = None

        # packages
        self.packages: Dict[str, Package] = {}

        # build tool
        self.built_by: BuildTool = None

        # devel path
        self.devel_space: str = None

        # install path
        self.install_space: str = None

        # source path
        self.source_space: str = None

        # build path
        self.build_space: str = None

        # logs path
        self.logs_space: str = None

    def get_build_package(self) -> Dict[str, Package]:
        return {k: v for (k, v) in self.packages.items() if v.enable}


class Param:
    def __init__(self):
        self._parser = argparse.ArgumentParser(description='')
        self._parser.add_argument(
            '-ws',
            '--work_space_dir',
            type=str,
            help="ros workspace path"
        )
        self._parser.add_argument(
            '-t',
            '--type',
            type=str,
            help='type'
        )
        self._args = self._parser.parse_args()

    def get_work_space_dir(self) -> str:
        return self._args.work_space_dir

    def get_type(self) -> str:
        return self._args.type


param = Param()


class Util:

    def __init__(self, ws_path: str = None):
        self._work_space = WorkSpace()
        self._work_space.path = ws_path
        self._init()

    def _init(self) -> None:
        # workspace path
        if None is self._work_space.path:
            self._work_space.path = param.get_work_space_dir()
            if None is self._work_space.path:
                raise

        # build space
        self._work_space.build_space = self._path_join(
            self._work_space.path, "build")
        if not self._path_exists(self._work_space.build_space):
            raise

        # ros version
        ros_distro: str = os.environ.get("ROS_DISTRO")
        if None is ros_distro:
            raise
        if ros_distro.upper() not in RosDistro.__members__:
            raise
        self._work_space.ros_distro = RosDistro[ros_distro.upper()]

        # build tool type
        if self._path_exists(
                self._path_join(self._work_space.path, "build/.built_by")):
            with open(self._path_join(self._work_space.path, "build/.built_by"), 'r') as file:
                file_content = file.readline().strip()
                if "catkin_make" == file_content:
                    self._work_space.built_by = BuildTool.CATKINMAKE
                elif "catkin build" == file_content:
                    self._work_space.built_by = BuildTool.CATKINTOOLS
        else:
            self._work_space.built_by = BuildTool.CMAKE

        # work space path
        if BuildTool.CATKINMAKE == self._work_space.built_by:
            self._parse_catkinmake_package()
        elif BuildTool.CATKINTOOLS == self._work_space.built_by:
            self._parse_catkintool_package()
        else:
            pass

        # all package
        exclude_dirs = []
        exclude_dirs.append(os.path.basename(self._work_space.build_space))
        exclude_dirs.append(os.path.basename(self._work_space.devel_space))
        exclude_dirs.append(os.path.basename(self._work_space.install_space))
        exclude_dirs.append("logs")
        exclude_dirs.append(".catkin_tools")
        for root, dirs, files in os.walk(
                self._work_space.path, followlinks=True):
            dirs[:] = [d for d in dirs if d not in exclude_dirs]
            for file in files:
                file_path = self._path_join(root, file)
                if file == 'package.xml':
                    name = self._parse_package_name(file_path)
                    if name not in self._work_space.packages.keys():
                        package = Package()
                        package.enable = False
                        package.name = name
                        package.path = root
                        self._work_space.packages[name] = package

                    self._work_space.packages[name].path = root
                    self._work_space.packages[name].software = self._is_software_package(
                        root)
                    self._work_space.packages[name].depend_package = self._parse_depend_package(
                        file_path)
                    self._work_space.packages[name].compile_commands_path = self._get_compile_commands_path(
                        name)

        return None

    def _parse_package_name(self, package_file: str) -> str:
        """
        parse package name
        :param package_file: packages.xml path
        :return: package name
        """
        tree = ET.parse(package_file)
        root = tree.getroot()
        name_tag = root.find("name")
        return None if name_tag is None else name_tag.text

    def _parse_depend_package(self, package_file: str) -> List[str]:
        """
        get package.xml all depend tag valua
        :param package_file: packages.xml path
        :return: [depend]
        """
        tree = ET.parse(package_file)
        root = tree.getroot()
        return [tag.text for tag in root.findall('.//depend')]

    def _parse_catkinmake_package(self) -> Dict[str, Package]:
        catkin_config_path = self._path_join(
            self._work_space.build_space, "catkin_make.cache")
        if not self._path_exists(catkin_config_path):
            raise

        with open(catkin_config_path, 'r') as file:
            line_num = 0
            for line in file:
                if 0 == line_num:
                    # package name
                    package_names = re.findall(r'\b\w+\b', line)
                    for name in package_names:
                        package = Package()
                        package.enable = True
                        package.name = name
                        if name not in self._work_space.packages.keys():
                            self._work_space.packages[name] = package
                elif 1 == line_num:
                    # space
                    prefixes = re.findall(r'PREFIX=([^ ]+)', line)
                    for space in prefixes:
                        if "devel" in space:
                            self._work_space.devel_space = space
                        elif "install" in space:
                            self._work_space.install_space = space
                else:
                    break
                line_num += 1

        return None

    def _parse_catkintool_package(self) -> Dict[str, Package]:
        catkin_config_path = self._path_join(
            self._work_space.build_space, ".catkin_tools.yaml")
        if not self._path_exists(catkin_config_path):
            raise

        with open(catkin_config_path, "r") as file:
            for line in file:
                if ':' in line:
                    key, value = line.strip().split(':', 1)
                    if "devel_space" == key.strip():
                        self._work_space.devel_space = value.strip()
                    elif "install_space" == key.strip():
                        self._work_space.install_space = value.strip()
                    elif "source_space" == key.strip():
                        self._work_space.source_space = value.strip()

        packages_path = self._path_join(
            self._work_space.path,
            ".catkin_tools/profiles/default/packages/")
        if not self._path_exists(packages_path):
            raise
        pacakges_folder = os.listdir(packages_path)
        for name in pacakges_folder:
            if self._path_is_dir(self._path_join(packages_path, name)
                                 ) and "catkin_tools_prebuild" != name:
                package = Package()
                package.enable = True
                package.name = name
                if name not in self._work_space.packages.keys():
                    self._work_space.packages[name] = package

        return None

    def _is_software_package(self, path: str) -> bool:
        for root, dirs, files in os.walk(path):
            for file in files:
                file_ext = os.path.splitext(file)[1]
                if file_ext == '.c' or file_ext == '.cc' or file_ext == '.cpp' or file_ext == '.cxx':
                    return True
                elif file_ext == '.h' or file_ext == '.hpp' or file_ext == '.py':
                    return True
                elif file_ext == '.py':
                    return True
        return False

    def _get_compile_commands_path(self, name: str) -> str:
        package = self._work_space.packages.get(name, None)
        if None is package:
            return None
        path = self._path_join(
            self._path_join(
                self._work_space.build_space,
                package.name),
            "compile_commands.json")
        return path if self._path_exists(path) else None

    def _path_exists(self, path: str) -> bool:
        return os.path.exists(path)

    def _path_is_dir(self, path: str):
        return os.path.isdir(path)

    def _path_is_link(self, path: str):
        return os.path.islink(path)

    def _path_join(self, a: str, b: str):
        return os.path.join(a, b)

    def debug_print(self) -> None:
        print("work space path: {}".format(self._work_space.path))
        print("work space built by: {}".format(self._work_space.built_by))
        print("work space build path: {}".format(self._work_space.build_space))
        print(
            "work space install path: {}".format(
                self._work_space.install_space))
        print("work space devle path: {}".format(self._work_space.devel_space))
        print(
            "work space source path: {}".format(
                self._work_space.source_space))
        print("work space package size: {}".format(
            len(self._work_space.packages)))
        for name, package in self._work_space.packages.items():
            print("  name: {}".format(name))
            print("  enable: {}".format(package.enable))
            print("  path: {}".format(package.path))
            print("  software: {}".format(package.software))
            print(
                "  compile_commands.json: {}".format(
                    package.compile_commands_path))
            print("  depend package: {}".format(package.depend_package))
            print("---")
        return None

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
        return self._work_space.built_by

    def is_work_space(self) -> bool:
        """
        判断传入的参数是否是Ros工作区
        :return:
        """
        return self._work_space.path is not None

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

    def get_build_package_list(self) -> Dict[str, Package]:
        """
        get build package
        :return:
        """
        if not self.is_work_space():
            return {}
        return {k: v for k, v in self._work_space.packages.items() if v.enable}

    def get_package(self, name: str) -> Package:
        """
        get package
        :param name: name
        :return: Package object
        """
        return self._work_space.packages.get(name, None)

    def get_package_include_path(self, name: str) -> str:
        package = self._work_space.packages.get(name, None)
        if None is package:
            return None
        include_path = self._path_join(package.path, "include")
        return include_path if self._path_exists(include_path) else None

    def generate_compile_commands(self, name: str, depend: str) -> str:
        package_target = self._work_space.packages.get(name, None)
        package_depend = self._work_space.packages.get(depend, None)
        if None is package_target or not self._path_exists(
                package_target.compile_commands_path) or None is package_depend:
            return None

        data = None
        with open(package_target.compile_commands_path, 'r') as f:
            data = json.load(f)

        for context in data:
            if isinstance(context, dict) and "command" in context.keys():
                command = context["command"]
                dros_package_name = 'DROS_PACKAGE_NAME=\\\"{}\\\"'.format(
                    package_target.name)
                insert_string = " -I{} ".format(
                    self.get_package_include_path(depend))
                if insert_string in command:
                    continue
                insert_index = command.find(
                    dros_package_name) + len(dros_package_name)
                modified_string = command[:insert_index] + \
                    insert_string + command[insert_index:]
            context["command"] = modified_string

        with open(package_target.compile_commands_path, 'w') as f:
            json.dump(data, f)

        return data


if __name__ == "__main__":
    util = Util(param.get_work_space_dir())
    util.debug_print()
