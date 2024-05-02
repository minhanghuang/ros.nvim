import os
import glob
import json
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
            self.generate_compile_commands()
        return None

    def generate_compile_commands(self) -> None:
        if not self._util.is_work_space() or self._util.has_compile_commands_file():
            return None

        compile_commands_files = glob.glob(
            self._ws_path +
            '/build/**/compile_commands.json',
            recursive=True
        )

        # 读取所有文件中的JSON数据
        all_json_data = []
        for file in compile_commands_files:
            with open(file, 'r') as f:
                data = json.load(f)
                all_json_data.extend(data)

        for package_name in self._util.get_build_package_list():
            package_path = self._util.get_package_path(package_name)
            if None is package_path:
                continue
            print(package_path)
            if None is not package_path and os.path.exists(package_path):
                # 将所有数据写入到单个compile_commands.json文件中
                with open(os.path.join(package_path, 'compile_commands.json'), 'w+') as f:
                    json.dump(all_json_data, f)

        return None


if __name__ == "__main__":
    obj = Clangd()
    obj.init()
    obj.run()
