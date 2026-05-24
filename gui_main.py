import os
import sys
import logging

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "libs"))

from config import SystemConfig
from gui import MainWindow


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
    logger.info("升降网箱模型控制系统 GUI 启动")

    config = SystemConfig()

    if len(sys.argv) > 1:
        config.serial.port = sys.argv[1]

    app = MainWindow(config)
    app.mainloop()


if __name__ == "__main__":
    main()
