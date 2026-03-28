Import("env")

board_config = env.BoardConfig()
# board_config.update("build.usb_product", "Custom Product Name")
# board_config.update("build.usb_manufacturer", "Custom Vendor")
board_config.update("build.usb_description", "Custom Product Description")


