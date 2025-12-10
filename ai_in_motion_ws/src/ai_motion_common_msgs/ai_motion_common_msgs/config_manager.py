"""
Configuration Manager

Implements a configuration management system for the AI in Motion project.
Handles loading, validating, and accessing configuration parameters.
"""

import os
import json
import yaml
from typing import Any, Dict, Optional, Union, List
from pathlib import Path


class ConfigManager:
    """
    Configuration manager for handling application settings and parameters.
    Supports multiple configuration sources and provides validation.
    """

    def __init__(self, config_paths: Optional[List[str]] = None):
        """
        Initialize the configuration manager.

        Args:
            config_paths: Optional list of configuration file paths to load
        """
        self._config: Dict[str, Any] = {}
        self._default_config: Dict[str, Any] = self._get_default_config()
        self._config_sources: List[str] = []

        if config_paths:
            for path in config_paths:
                self.load_config(path)

        # Load environment variables
        self._load_environment_config()

    def _get_default_config(self) -> Dict[str, Any]:
        """
        Get the default configuration values.

        Returns:
            Dictionary with default configuration values
        """
        return {
            # General settings
            "app_name": "AI in Motion",
            "version": "1.0.0",
            "environment": "development",  # development, staging, production

            # ROS 2 settings
            "ros": {
                "domain_id": 0,
                "use_sim_time": True,
                "log_level": "INFO",
                "node_namespace": "",
            },

            # Simulation settings
            "simulation": {
                "engine": "isaac_sim",  # isaac_sim, gazebo, unity
                "real_time_factor": 1.0,
                "physics_update_rate": 500,  # Hz
                "rendering_update_rate": 60,  # Hz
            },

            # Hardware settings
            "hardware": {
                "platform": "simulated",  # simulated, jetson, real_robot
                "connection_timeout": 10.0,  # seconds
                "max_joint_velocity": 2.0,  # rad/s
                "max_joint_effort": 100.0,  # Nm
            },

            # AI/ML settings
            "ai": {
                "openai_api_key": "",
                "whisper_model": "whisper-1",
                "max_audio_duration": 30,  # seconds
                "voice_confidence_threshold": 0.7,
            },

            # Navigation settings
            "navigation": {
                "max_linear_velocity": 0.5,  # m/s
                "max_angular_velocity": 0.5,  # rad/s
                "min_distance_to_goal": 0.1,  # meters
                "path_tolerance": 0.2,  # meters
                "rotation_tolerance": 0.1,  # radians
            },

            # Performance settings
            "performance": {
                "max_cpu_usage": 80.0,  # percentage
                "min_memory_mb": 1024,  # MB
                "update_rate": 50.0,  # Hz
                "max_loop_time_ms": 100,  # milliseconds
            },

            # Logging settings
            "logging": {
                "level": "INFO",
                "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
                "file_path": "logs/app.log",
                "max_file_size_mb": 10,
                "backup_count": 5,
            },

            # Paths and directories
            "paths": {
                "data_dir": "./data",
                "config_dir": "./config",
                "models_dir": "./models",
                "logs_dir": "./logs",
                "temp_dir": "./temp",
            }
        }

    def load_config(self, config_path: str) -> bool:
        """
        Load configuration from a file.

        Args:
            config_path: Path to the configuration file (JSON or YAML)

        Returns:
            True if loading was successful, False otherwise
        """
        path = Path(config_path)

        if not path.exists():
            print(f"Configuration file not found: {config_path}")
            return False

        try:
            with open(path, 'r', encoding='utf-8') as file:
                if path.suffix.lower() in ['.yaml', '.yml']:
                    config_data = yaml.safe_load(file)
                elif path.suffix.lower() == '.json':
                    config_data = json.load(file)
                else:
                    print(f"Unsupported config file format: {path.suffix}")
                    return False

            if config_data:
                # Deep merge the loaded config with existing config
                self._deep_merge(self._config, config_data)
                self._config_sources.append(config_path)
                return True

        except Exception as e:
            print(f"Error loading config from {config_path}: {e}")
            return False

        return False

    def _load_environment_config(self):
        """
        Load configuration from environment variables.
        Environment variables override file-based configuration.
        """
        env_mapping = {
            'AI_IN_MOTION_ENVIRONMENT': 'environment',
            'AI_IN_MOTION_ROS_DOMAIN_ID': 'ros.domain_id',
            'AI_IN_MOTION_USE_SIM_TIME': 'ros.use_sim_time',
            'AI_IN_MOTION_SIM_ENGINE': 'simulation.engine',
            'AI_IN_MOTION_OPENAI_API_KEY': 'ai.openai_api_key',
            'AI_IN_MOTION_MAX_LINEAR_VELOCITY': 'navigation.max_linear_velocity',
            'AI_IN_MOTION_LOG_LEVEL': 'logging.level',
        }

        for env_var, config_key in env_mapping.items():
            env_value = os.getenv(env_var)
            if env_value is not None:
                # Convert string values to appropriate types
                if config_key.endswith('.domain_id') or config_key.endswith('.physics_update_rate'):
                    env_value = int(env_value)
                elif config_key.endswith('.use_sim_time'):
                    env_value = env_value.lower() in ('true', '1', 'yes', 'on')
                elif any(keyword in config_key for keyword in ['_rate', '_velocity', '_time', '_factor', '_threshold']):
                    env_value = float(env_value)

                self.set_config_value(config_key, env_value)

    def get_config_value(self, key: str, default: Any = None) -> Any:
        """
        Get a configuration value using dot notation.

        Args:
            key: Configuration key using dot notation (e.g., 'ros.domain_id')
            default: Default value if key is not found

        Returns:
            Configuration value or default
        """
        keys = key.split('.')
        value = self._config

        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                # Try to get from default config
                default_value = self._get_nested_value(self._default_config, key)
                return default_value if default_value is not None else default

        return value

    def set_config_value(self, key: str, value: Any):
        """
        Set a configuration value using dot notation.

        Args:
            key: Configuration key using dot notation (e.g., 'ros.domain_id')
            value: Value to set
        """
        keys = key.split('.')
        config = self._config

        for k in keys[:-1]:
            if k not in config:
                config[k] = {}
            config = config[k]

        config[keys[-1]] = value

    def _get_nested_value(self, config: Dict[str, Any], key: str) -> Any:
        """
        Get a nested value from a configuration dictionary using dot notation.

        Args:
            config: Configuration dictionary
            key: Key using dot notation

        Returns:
            Value or None if not found
        """
        keys = key.split('.')
        value = config

        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return None

        return value

    def _deep_merge(self, base: Dict[str, Any], update: Dict[str, Any]):
        """
        Deep merge update dictionary into base dictionary.

        Args:
            base: Base dictionary to update
            update: Dictionary with updates
        """
        for key, value in update.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self._deep_merge(base[key], value)
            else:
                base[key] = value

    def get_all_config(self) -> Dict[str, Any]:
        """
        Get all configuration values.

        Returns:
            Complete configuration dictionary
        """
        # Return a copy to prevent external modification
        import copy
        return copy.deepcopy(self._config)

    def validate_config(self) -> List[str]:
        """
        Validate the current configuration.

        Returns:
            List of validation errors, empty if valid
        """
        errors = []

        # Validate ROS settings
        ros_domain_id = self.get_config_value('ros.domain_id', 0)
        if not isinstance(ros_domain_id, int) or ros_domain_id < 0 or ros_domain_id > 101:
            errors.append("ros.domain_id must be an integer between 0 and 101")

        # Validate simulation settings
        sim_engine = self.get_config_value('simulation.engine', 'isaac_sim')
        valid_engines = ['isaac_sim', 'gazebo', 'unity', 'simulated']
        if sim_engine not in valid_engines:
            errors.append(f"simulation.engine must be one of {valid_engines}")

        # Validate AI settings
        api_key = self.get_config_value('ai.openai_api_key', '')
        if not api_key and self.get_config_value('environment', 'development') == 'production':
            errors.append("ai.openai_api_key is required in production environment")

        # Validate performance settings
        max_cpu = self.get_config_value('performance.max_cpu_usage', 80.0)
        if not 0 <= max_cpu <= 100:
            errors.append("performance.max_cpu_usage must be between 0 and 100")

        # Validate navigation settings
        max_lin_vel = self.get_config_value('navigation.max_linear_velocity', 0.5)
        if max_lin_vel <= 0:
            errors.append("navigation.max_linear_velocity must be positive")

        return errors

    def save_config(self, config_path: str, format: str = 'yaml') -> bool:
        """
        Save the current configuration to a file.

        Args:
            config_path: Path to save the configuration
            format: Format to save in ('yaml' or 'json')

        Returns:
            True if saving was successful, False otherwise
        """
        try:
            path = Path(config_path)
            path.parent.mkdir(parents=True, exist_ok=True)

            with open(path, 'w', encoding='utf-8') as file:
                if format.lower() == 'json':
                    json.dump(self._config, file, indent=2, ensure_ascii=False)
                else:  # default to YAML
                    yaml.dump(self._config, file, default_flow_style=False, allow_unicode=True)

            return True
        except Exception as e:
            print(f"Error saving config to {config_path}: {e}")
            return False

    def reset_to_defaults(self):
        """Reset all configuration values to defaults."""
        self._config = {}
        self._config_sources = []

    def get_config_sources(self) -> List[str]:
        """
        Get the list of configuration sources that were loaded.

        Returns:
            List of configuration file paths
        """
        return self._config_sources.copy()


# Global configuration manager instance
_config_manager: Optional[ConfigManager] = None


def get_config_manager(config_paths: Optional[List[str]] = None) -> ConfigManager:
    """
    Get the global configuration manager instance.

    Args:
        config_paths: Optional list of configuration file paths to load

    Returns:
        Configuration manager instance
    """
    global _config_manager
    if _config_manager is None:
        _config_manager = ConfigManager(config_paths)
    return _config_manager


def get_config_value(key: str, default: Any = None) -> Any:
    """
    Get a configuration value using the global configuration manager.

    Args:
        key: Configuration key using dot notation
        default: Default value if key is not found

    Returns:
        Configuration value or default
    """
    config_manager = get_config_manager()
    return config_manager.get_config_value(key, default)


def set_config_value(key: str, value: Any):
    """
    Set a configuration value using the global configuration manager.

    Args:
        key: Configuration key using dot notation
        value: Value to set
    """
    config_manager = get_config_manager()
    config_manager.set_config_value(key, value)


def validate_config() -> List[str]:
    """
    Validate the current configuration using the global configuration manager.

    Returns:
        List of validation errors, empty if valid
    """
    config_manager = get_config_manager()
    return config_manager.validate_config()


def load_config(config_path: str) -> bool:
    """
    Load configuration from a file using the global configuration manager.

    Args:
        config_path: Path to the configuration file

    Returns:
        True if loading was successful, False otherwise
    """
    config_manager = get_config_manager()
    return config_manager.load_config(config_path)