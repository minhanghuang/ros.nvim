import glob
import os
import re
from typing import Dict

from base import Util, param


class Clangd:
    def __init__(self) -> None:
        self._ws_path = param.get_work_space_dir()
        self._ok = False
        try:
            self._util = Util(self._ws_path)
        except Exception as _:
            return None
        self._ok = True

    def run(self) -> None:
        if not self._ok:
            return None
        if "compile_commands" == param.get_type():
            self.generate_compile_commands()
        return None

    def generate_compile_commands(self) -> None:
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
                    if depend_package == None:
                        return None
                    if None is not depend_package and depend_package.software:
                        self._util.generate_compile_commands(name, depend_name)

        compile_commands_files = glob.glob(
            self._ws_path + "/build/**/compile_commands.json", recursive=True
        )

        packages = {}
        pattern = r"build/(\w+)/compile_commands.json"
        for compile_commands_file in compile_commands_files:
            match = re.search(pattern, compile_commands_file)
            if (
                match
                and isinstance(compile_commands_file, str)
                and bool(compile_commands_file.strip())
            ):
                packages[match.group(1)] = compile_commands_file

        for name, package in self._util.get_build_package_list().items():
            if None is package.path or name not in packages:
                continue
            link_name = os.path.join(package.path, "compile_commands.json")
            source = packages[name]

            if os.path.islink(link_name):
                # 软连接
                target_path = os.readlink(link_name)
                if not os.path.isabs(target_path):
                    # 如果不是绝对路径, 先转换为绝对路径
                    target_path = os.path.join(os.path.dirname(link_name), target_path)
                if not os.path.exists(target_path):
                    # 软连接失效
                    try:
                        os.remove(link_name)
                    except Exception as _:
                        pass
                    os.symlink(source, link_name)
                else:
                    # 软连接已经存在
                    if source != target_path:
                        # 软连接与目标连接不一致, 更新软连接
                        os.remove(link_name)
                        os.symlink(source, link_name)
                    else:
                        # 软连接与目标连接一致, 不更新
                        pass
            else:
                # 不是软连接
                if not os.path.exists(link_name):
                    # compile_commands.json不存在
                    os.symlink(source, link_name)
                else:
                    # 文件存在, 不更新
                    pass

        return None


if __name__ == "__main__":
    obj = Clangd()
    obj.run()
