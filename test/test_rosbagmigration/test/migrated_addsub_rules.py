class update_test_rosbagmigration_MigratedAddSub_c5d68a52143661d05909248952fc11f1(MessageUpdateRule):
	old_type = "test_rosbagmigration/MigratedAddSub"
	old_full_text = """
Simple field1
================================================================================
MSG: test_rosbagmigration/Simple
int32 field1 #42
"""

	new_type = "test_rosbagmigration/MigratedAddSub"
	new_full_text = """
Simple field1
Simple field2
================================================================================
MSG: test_rosbagmigration/Simple
int32 field1 #42
"""

	order = 0
	migrated_types = [("Simple","Simple"),]

	valid = True

	def update(self, old_msg, new_msg):
		self.migrate(old_msg.field1, new_msg.field1)
		new_msg.field2 = self.get_new_class('Simple')(42)

