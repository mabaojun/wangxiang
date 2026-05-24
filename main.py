import logging
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "libs"))

from config import SystemConfig
from cli import CLI


def setup_logging(level=logging.INFO):
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler("cage_control.log", encoding="utf-8"),
        ],
    )


def main():
    setup_logging()
    logger = logging.getLogger(__name__)
    logger.info("升降网箱模型控制系统启动")

    config = SystemConfig()

    if len(sys.argv) > 1:
        config.serial.port = sys.argv[1]
        logger.info(f"使用命令行指定串口: {config.serial.port}")

    app = CLI(config)
    try:
        app.start()
    except Exception as e:
        logger.critical(f"系统异常: {e}", exc_info=True)
        print(f"\n系统异常: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
