import pytest
from s3e_pipeline.mixed_pgo import settings

def test_cpu_settings_keep_their_historical_fingerprint():
    assert not any(k.startswith('gpu_') for k in settings({'factor':'gicp'}))

def test_glim_gpu_settings_and_invalid_intervals():
    cfg=settings({'enabled':True,'factor':'vgicp_gpu'})
    assert cfg['gpu_voxel_levels']==2 and cfg['gpu_voxel_m']==.5 and cfg['gpu_voxel_max_m']==1.
    for bad in [{'gpu_voxel_levels':0},{'gpu_voxel_levels':True},{'gpu_voxel_m':float('nan')},
                {'gpu_voxel_max_m':.1},{'gpu_distance_max_m':5.},{'gpu_voxel_scaling':1.}]:
        with pytest.raises(ValueError):settings(dict(factor='vgicp_gpu',**bad))
    with pytest.raises(ValueError):settings({'factor':'gicp','gpu_voxel_levels':2})


def test_selected_binary_and_provenance_share_overlay_resolution(tmp_path, monkeypatch):
    from s3e_pipeline.dpgo import native_overlay
    monkeypatch.delenv('CBS_OVERLAY', raising=False)
    cfg = {'registration_factors': {'factor': 'vgicp_gpu'}}
    assert native_overlay(tmp_path, cfg) == tmp_path / '.ros2/dpgo-gpu-install'
    assert native_overlay(tmp_path) == tmp_path / '.ros2/dpgo-install'
    explicit = tmp_path / 'isolated-install'
    monkeypatch.setenv('CBS_OVERLAY', str(explicit))
    assert native_overlay(tmp_path, cfg) == explicit


def test_new_default_gpu_preserves_explicit_cpu_pipeline():
    from pathlib import Path
    import yaml
    root = Path(__file__).parents[1] / 'configs'
    gpu = yaml.safe_load((root / 'default_pipeline.yaml').read_text())
    cpu = yaml.safe_load((root / 'cpu_gicp_pipeline.yaml').read_text())
    assert gpu['dpgo']['registration_factors']['factor'] == 'vgicp_gpu'
    assert cpu['dpgo']['registration_factors']['factor'] == 'gicp'
    for field in ('odometry', 'backend', 'loops', 'pgo', 'evaluation'):
        assert gpu[field] == cpu[field]
    assert settings(gpu['dpgo']['registration_factors'])['gpu_voxel_levels'] == 2
