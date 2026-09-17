"""Configuration loading and validation for the OpenSim IK application."""

import pathlib

import tomli


class Config:
    """Load application settings from a TOML configuration file."""

    def __init__(self, args):
        with open(args.config, "rb") as config_file:
            self.data = tomli.load(config_file)
        self.home_dir = pathlib.Path(__file__).parent.resolve()
        self.initialize()
        self.validate_online_mode(args)

    def initialize(self):
        # Required parameters
        self.validate_required_keys(
            [
                "offline",
                "accuracy",
                "constraint_var",
                "sensors",
                "base_imu",
                "base_imu_axis",
                "output_filename",
            ]
        )
        self.visualize = self.data["visualize"]
        self.offline = self.data["offline"]
        self.rate = self.data["rate"]
        self.accuracy = self.data["accuracy"] if not self.data["offline_dev"] else 1e-5
        self.constraint_var = (
            self.data["constraint_var"] if not self.data["offline_dev"] else 20.0
        )
        self.sensors = self.data["sensors"]
        self.base_imu = self.data["base_imu"]
        self.base_imu_axis = self.data["base_imu_axis"]
        self.uncal_model = (
            "Rajagopal_2015.osim"
            if not self.data["clamped"]
            else "clamped_Rajagopal_2015.osim"
        )
        self.uncal_model_filename = self.home_dir / self.uncal_model
        self.model_filename = self.home_dir / ("calibrated_" + self.uncal_model)
        self.offline_data_folder = self.home_dir / "offline"
        self.sto_filename = str(self.home_dir / "temp_file.sto")
        self.save_dir_init = self.home_dir / "recordings"
        self.offline_data_name = self.data["offline_data_name"] if self.offline else None
        self.log_angle = self.data["log_angle"]
        self.log_time = self.data["log_time"] if not self.offline else False
        self.output_filename = self.data["output_filename"]
        self.time_filename = self.data["time_filename"]
        self.logging_level = self.data["logging_level"]
        self.offline_padded = self.data["offline_padded"]
        self.datadrop_comp = self.data["datadrop_comp"]
        self.ave_quat = self.data["ave_quat"]

    def validate_required_keys(self, required_keys):
        missing_keys = [key for key in required_keys if key not in self.data]
        if missing_keys:
            raise KeyError(
                f"Missing required configuration keys: {', '.join(missing_keys)}"
            )

    def validate_online_mode(self, args):
        if not args.address and not self.offline:
            raise ValueError("Online mode requires an address.")
