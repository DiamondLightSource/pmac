from malcolm.yamlutil import make_block_creator, check_yaml_names

pmac_runnable_block = make_block_creator(__file__, "pmac_runnable_block.yaml")
pmac_test_scan = make_block_creator(__file__, "pmac_test_scan.yaml")
dummy_trigger_block = make_block_creator(__file__, "dummy_trigger_block.yaml")


__all__ = check_yaml_names(globals())

