"""Load installed or explicitly selected configuration without absolute paths."""
from pathlib import Path
from typing import Any
import argparse
import logging
import yaml


def load_config(path: Path) -> dict[str, Any]:
    with Path(path).open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected a mapping')
    return data


def class_mapping(config: dict[str, Any]) -> dict[str, int]:
    classes = config.get('classes', [])
    if not classes or any(not isinstance(c, str) for c in classes) or len(set(classes)) != len(classes):
        raise ValueError('Classes must be unique nonempty strings')
    return {name: index for index, name in enumerate(classes)}


def validate_directory(directory: Path) -> list[str]:
    configs = {name: load_config(directory / f'{name}.yaml') for name in
               ('robots', 'disaster_classes', 'mrta', 'experiments')}
    mapping = class_mapping(configs['disaster_classes'])
    robots = configs['robots']['robots']
    ids = [r['robot_id'] for r in robots]
    if len(set(ids)) != len(ids):
        raise ValueError('Duplicate robot IDs')
    for robot in robots:
        if not 0 <= robot['battery'] <= 100 or robot['speed_mps'] <= 0:
            raise ValueError('Invalid robot battery or speed')
    mrta = configs['mrta']
    if mrta['algorithm'] not in mrta['strategies'] or mrta['distance_normalizer_m'] <= 0:
        raise ValueError('Invalid allocation configuration')
    for experiment in configs['experiments']['experiments']:
        for key in ('failure_probability', 'communication_failure_probability'):
            if not 0 <= experiment[key] <= 1:
                raise ValueError(f'Invalid {key}')
    capabilities = {c for robot in robots for c in robot['capabilities']}
    warnings = []
    for name in mapping:
        rule = configs['disaster_classes']['rules'][name]
        capability = rule['required_capability']
        if capability and capability not in capabilities:
            warnings.append(f'{name}: no robot supports {capability}; tasks remain unassigned')
    return warnings


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-dir', type=Path)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    try:
        directory = args.config_dir
        if directory is None:
            from ament_index_python.packages import get_package_share_directory
            directory = Path(get_package_share_directory('disaster_mrta')) / 'config'
        for warning in validate_directory(directory):
            logging.warning(warning)
        logging.info('Configuration validated: %s', directory)
    except (OSError, ValueError, KeyError, ImportError, yaml.YAMLError) as error:
        logging.error('Configuration validation failed: %s', error)
        raise SystemExit(1) from error


if __name__ == '__main__':
    main()
