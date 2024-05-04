import os
import re
import glob
import json
import shutil
from base import param
from base import Util


class Clangd:

    def __init__(self) -> None:
        self._ws_path = param.get_work_space_dir()
        self._util: Util = None
        self._ok = False

    def init(self) -> None:
        try:
            self._util = Util(self._ws_path)
        except Exception as _:
            return None
        self._ok = True
        return None

    def run(self) -> None:
        if not self._ok:
            return None
        if "compile_commands" == param.get_type():
            self.generate_compile_commands2()
        return None

    def generate_compile_commands(self) -> None:
        if not self._util.is_work_space() or self._util.has_compile_commands_file():
            return None

        compile_commands_files = glob.glob(
            self._ws_path +
            '/build/**/compile_commands.json',
            recursive=True
        )

        packages = {}
        pattern = r'build/(\w+)/compile_commands.json'
        for compile_commands_file in compile_commands_files:
            match = re.search(pattern, compile_commands_file)
            if match:
                packages[match.group(1)] = compile_commands_file

        for name, package in self._util.get_build_package_list().items():
            if None is package.path or name not in packages:
                continue
            shutil.copyfile(
                packages[name],
                os.path.join(
                    package.path,
                    "compile_commands.json"))

        return None

    def generate_compile_commands2(self) -> None:
        if not self._util.is_work_space() or self._util.has_compile_commands_file():
            return None

        packages = self._util.get_build_package_list()
        for name, package in packages.items():
            if not package.software or None is package.compile_commands_path:
                continue
            # is software package
            for depend_name in package.depend_package:
                if depend_name in packages.keys():
                    depend_package = self._util.get_package(depend_name)
                    if None is not depend_package and depend_package.software:
                        self._util.generate_compile_commands(name, depend_name)

        compile_commands_files = glob.glob(
            self._ws_path +
            '/build/**/compile_commands.json',
            recursive=True
        )

        packages = {}
        pattern = r'build/(\w+)/compile_commands.json'
        for compile_commands_file in compile_commands_files:
            match = re.search(pattern, compile_commands_file)
            if match:
                packages[match.group(1)] = compile_commands_file

        for name, package in self._util.get_build_package_list().items():
            if None is package.path or name not in packages:
                continue
            shutil.copyfile(
                packages[name],
                os.path.join(
                    package.path,
                    "compile_commands.json"))

        return None


if __name__ == "__main__":
    obj = Clangd()
    obj.init()
    obj.run()
