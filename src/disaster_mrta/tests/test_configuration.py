from pathlib import Path
import pytest
from disaster_mrta.config_loader import class_mapping, validate_directory


def test_mapping_follows_config_order():
    assert class_mapping({'classes': ['Fire', 'Water']}) == {'Fire': 0, 'Water': 1}
    assert class_mapping({'classes': ['Water', 'Fire']})['Fire'] == 1


def test_duplicate_classes_rejected():
    with pytest.raises(ValueError):
        class_mapping({'classes': ['Fire', 'Fire']})


def test_fleet_supports_all_visual_task_capabilities():
    warnings = validate_directory(Path(__file__).parents[1] / 'config')
    assert warnings == []
