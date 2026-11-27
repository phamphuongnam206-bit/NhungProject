from configparser import ConfigParser
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory


def main():
    root = Path(__file__).resolve().parents[1]
    source = Path(sys.argv[1]).resolve()
    if source.parent != root / "src" or source.suffix != ".cpp" or not source.is_file():
        sys.exit("Hãy mở file .cpp trong thư mục src trước khi chọn nạp.")

    config_path = root / "platformio.ini"
    config = ConfigParser(interpolation=None)
    config.read(config_path)
    pio = Path(sys.executable).with_name("pio")
    print(f"Nạp src/{source.name} qua ST-LINK", flush=True)
    for section in config.sections():
        filters = config.get(section, "build_src_filter", fallback="").split()
        sources = [item for item in filters if item.startswith("+<")]
        if section.startswith("env:") and sources[:1] == [f"+<{source.name}>"]:
            return subprocess.call([pio, "run", "-e", section[4:], "-t", "upload"], cwd=root)

    # A new standalone sketch can use the shared board settings.
    with TemporaryDirectory() as directory:
        local_config = Path(directory) / "platformio.ini"
        with local_config.open("wb") as output:
            output.write(config_path.read_bytes())
            output.write(f"\n[env:active_file]\nbuild_src_filter = -<*> +<{source.name}>\n".encode())
        return subprocess.call([pio, "run", "-c", local_config, "-e", "active_file", "-t", "upload"], cwd=root)


if __name__ == "__main__":
    sys.exit(main())
