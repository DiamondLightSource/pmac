# Treat all division as float division even in python2
from __future__ import division

from annotypes import add_call_types, Anno, TYPE_CHECKING

from malcolm.core import APartName, Block, Attribute, Context, PartRegistrar
from malcolm.modules import builtin, scanning


if TYPE_CHECKING:
    from typing import List, Tuple, Dict

# Pull re-used annotypes into our namespace in case we are subclassed
APartName = APartName
AMri = builtin.parts.AMri


class DummyTriggerPart(builtin.parts.ChildPart):
    """Part for providing a triggering mode for the pmac in system tests
    this allows testing of the triggering modes without a PandA
    """

    def __init__(self, name, mri):
        # type: (APartName, AMri) -> None
        super(DummyTriggerPart, self).__init__(
            name, mri, initial_visibility=True, stateful=False)
        self.test = True

    def setup(self, registrar):
        # type: (PartRegistrar) -> None
        super(DummyTriggerPart, self).setup(registrar)
        # Hooks
        registrar.hook(scanning.hooks.ReportStatusHook, self.report_status)

    @add_call_types
    def report_status(self, context):
        # type: (scanning.hooks.AContext) -> scanning.hooks.UInfos
        child = context.block_view(self.mri)
        # Work out if we need the motor controller to send start of row triggers
        # or no triggers
        if child.rowTrigger.value == 'None':
            trigger = scanning.infos.MotionTrigger.NONE
        elif child.rowTrigger.value == 'Row Gate':
            trigger = scanning.infos.MotionTrigger.ROW_GATE
        else:
            trigger = scanning.infos.MotionTrigger.EVERY_POINT
        info = scanning.infos.MotionTriggerInfo(trigger)
        return info
