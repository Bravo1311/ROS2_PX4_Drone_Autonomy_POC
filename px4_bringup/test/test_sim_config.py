from pathlib import Path

import pytest

from px4_bringup import sim_config

ENV = ('PX4_GZ_WORLD', 'PX4_VEHICLE', 'PX4_GZ_INSTANCE', 'SIM_ENV_FILE')
SHIPPED = Path(__file__).resolve().parent.parent / 'config' / 'sim.env'
CUSTOM = (
    '# a comment\n'
    'PX4_GZ_WORLD=vio_rich\n'
    'PX4_VEHICLE=gz_x500_vio\n'
    'PX4_GZ_INSTANCE=2\n'
)


@pytest.fixture(autouse=True)
def clean_env(monkeypatch):
    for name in ENV:
        monkeypatch.delenv(name, raising=False)
    sim_config.file_values.cache_clear()
    yield
    sim_config.file_values.cache_clear()


@pytest.fixture
def custom_file(tmp_path, monkeypatch):
    path = tmp_path / 'sim.env'
    path.write_text(CUSTOM)
    monkeypatch.setenv('SIM_ENV_FILE', str(path))
    return path


def test_the_shipped_file_has_every_required_setting():
    values = sim_config.parse_env_file(SHIPPED.read_text())
    assert all(values.get(key) for key in sim_config.REQUIRED)


def test_values_come_from_the_file(custom_file):
    assert sim_config.world_name() == 'vio_rich'
    assert sim_config.model_name() == 'x500_vio'
    assert sim_config.drone_name() == 'x500_vio_2'


def test_the_shipped_default_is_the_original_setup():
    assert sim_config.world_name() == 'walls'
    assert sim_config.drone_name() == 'x500_mono_cam_down_0'


def test_an_environment_variable_beats_the_file(custom_file, monkeypatch):
    monkeypatch.setenv('PX4_GZ_WORLD', 'walls')
    assert sim_config.world_name() == 'walls'
    assert sim_config.vehicle_target() == 'gz_x500_vio'      # still from the file


def test_an_empty_variable_falls_back_to_the_file(custom_file, monkeypatch):
    monkeypatch.setenv('PX4_GZ_WORLD', '')
    assert sim_config.world_name() == 'vio_rich'


def test_explicit_arguments_beat_everything(custom_file):
    assert sim_config.drone_name('gz_x500_lidar_2d', 1) == 'x500_lidar_2d_1'


def test_a_target_without_the_gz_prefix_is_kept_as_is():
    assert sim_config.model_name('x500') == 'x500'


def test_a_missing_setting_is_reported(tmp_path, monkeypatch):
    path = tmp_path / 'sim.env'
    path.write_text('PX4_GZ_WORLD=walls\n')
    monkeypatch.setenv('SIM_ENV_FILE', str(path))
    with pytest.raises(ValueError, match='PX4_VEHICLE'):
        sim_config.world_name()


def test_parsing_ignores_comments_and_strips_quotes():
    parsed = sim_config.parse_env_file('# c\n\nA=1\nB="two words"\nC = 3\n')
    assert parsed == {'A': '1', 'B': 'two words', 'C': '3'}
