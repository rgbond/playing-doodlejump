import os
import sqlite3

drop_frame_tbl_sql = 'DROP TABLE IF EXISTS frames'
drop_rollouts_tbl_sql = 'DROP TABLE IF EXISTS rollouts'
schema_sql = '''
    CREATE TABLE frames(
    k INTEGER PRIMARY KEY,
    in_use INTEGER NON NULL,
    rollout INTEGER,
    frame INTEGER,
    file_name VARCHAR,
    score INTEGER,
    action INTEGER,
    reward INTEGER
    )
'''
rollout_schema_sql = '''
    CREATE TABLE rollouts(
    r INTEGER PRIMARY key,
    rewards INTEGER NON NULL,
    used INTEGER NON NULL
    )
'''
index_sql = 'CREATE INDEX rollout_index ON frames(rollout)'
init_row_sql = 'INSERT INTO frames(in_use) VALUES(0)'
get_space_sql = 'SELECT COUNT(*) FROM frames WHERE frames.in_use = 0'
next_rollout_sql = 'SELECT MAX(rollout) FROM frames'
min_rollout_sql = 'SELECT MIN(rollout) FROM frames'
drop_rollout_sql = 'UPDATE frames SET in_use = 0 where rollout = ?'
update_rff_sql = '''UPDATE frames SET in_use = 1,
                                      rollout = ?,
                                      frame = ?,
                                      file_name = ?,
                                      score = 0,
                                      action = 0,
                                      reward = 0
                                      WHERE k = ?
                 '''
update_action_sql = 'UPDATE frames SET action = ? WHERE frame = ? and rollout = ?'
update_score_sql = 'UPDATE frames SET score = ? WHERE frame = ? and rollout = ?'
update_reward_sql = 'UPDATE frames SET reward = ? WHERE frame = ? and rollout = ?'
next_k_sql = 'SELECT MIN(k) FROM frames WHERE frames.in_use = 0'
dump_sql = 'SELECT * FROM frames'
list_rollouts_sql = 'SELECT DISTINCT(rollout) FROM frames WHERE in_use == 1 ORDER BY rollout;'
get_rollout_fn_score_sql = 'SELECT file_name, score FROM frames WHERE rollout = ? ORDER BY frame'
get_rollout_fn_score_action_sql = 'SELECT file_name, score, action FROM frames WHERE rollout = ? ORDER BY frame'
get_rollout_frame_score_action_sql = 'SELECT frame, score, action FROM frames WHERE rollout = ? ORDER BY frame'
get_rollout_fn_action_sql = 'SELECT file_name, action FROM frames WHERE rollout = ? ORDER BY frame'
get_rollout_fn_action_rew_sql = 'SELECT file_name, action, reward FROM frames WHERE rollout = ? ORDER BY frame'
get_rollout_fn_reward_sql = 'SELECT file_name, reward FROM frames WHERE rollout = ? ORDER BY frame'

insert_rollout_rec_sql = 'INSERT INTO rollouts(r, rewards, used) VALUES(?, 0, 0)'
delete_rollout_rec_sql = 'DELETE FROM rollouts WHERE r=?'
update_rollout_rec_reward_sql = 'UPDATE rollouts SET rewards = 1 where r = ?'
update_rollout_rec_used_sql = 'UPDATE rollouts SET used = 1 where used = 0'
list_unrewarded_rollouts_sql = 'select r from rollouts where rewards = 0'
list_used_rollouts_sql  = 'select r from rollouts where used = ?'

class db(object):
    def __init__(self, db_path, setup=False):
        if not os.path.isfile(db_path):
            setup = True
        self.db = sqlite3.connect(db_path)
        self.c = self.db.cursor()
        if setup:
            self.mk_db(db_path)
        self.c.execute(get_space_sql)
        self.free_rows = self.c.fetchone()[0]
        self.c.execute(next_rollout_sql)
        self.rollout = self.c.fetchone()[0]
        if self.rollout is None:
            self.rollout = 1
        else:
            self.rollout += 1

    def commit(self):
        self.db.commit()

    def mk_db(self, db_path):
        self.nrows = 30000
        self.c.execute(drop_frame_tbl_sql)
        self.c.execute(drop_rollouts_tbl_sql)
        self.c.execute(schema_sql)
        self.c.execute(rollout_schema_sql)
        self.c.execute(index_sql)
        for i in range(self.nrows):
            self.c.execute(init_row_sql)
        self.db.commit()

    def list_rollouts(self):
        self.c.execute(list_rollouts_sql)
        return self.c.fetchall()

    def get_rollout_fn_score(self, r):
        self.c.execute(get_rollout_fn_score_sql, (r,))
        return self.c.fetchall()

    def get_rollout_fn_action(self, r):
        self.c.execute(get_rollout_fn_action_sql, (r,))
        return self.c.fetchall()

    def get_rollout_fn_action_rew(self, r):
        self.c.execute(get_rollout_fn_action_rew_sql, (r,))
        return self.c.fetchall()

    def get_rollout_fn_reward(self, r):
        self.c.execute(get_rollout_fn_reward_sql, (r,))
        return self.c.fetchall()

    def get_rollout_fn_score_action(self, r):
        self.c.execute(get_rollout_fn_score_action_sql, (r,))
        return self.c.fetchall()

    def get_rollout_frame_score_action(self, r):
        self.c.execute(get_rollout_frame_score_action_sql, (r,))
        return self.c.fetchall()

    def get_rollout(self):
        return self.rollout

    def advance_rollout(self):
        self.rollout += 1

    def make_space(self):
        self.c.execute(min_rollout_sql)
        to_remove = self.c.fetchone()[0]
        self.c.execute(drop_rollout_sql, (to_remove,))
        self.c.execute(get_space_sql)
        self.free_rows = self.c.fetchone()[0]
        self.c.execute(delete_rollout_rec_sql, (to_remove,))
        self.db.commit()
        
    def get_free_row(self):
        if self.free_rows == 0:
            self.make_space()
        self.c.execute(next_k_sql)
        k = self.c.fetchone()[0]
        return k

    def save_record(self, frame, path, rollout=-1):
        if rollout == -1:
            rollout = self.rollout
        self.last_k = self.get_free_row()
        self.c.execute(update_rff_sql, (rollout, frame, path, self.last_k))
        self.free_rows -= 1

    def update_action(self, action, frame, rollout=-1):
        if rollout == -1:
            rollout = self.rollout
        self.c.execute(update_action_sql, (action, frame, rollout)) 

    def update_score(self, score, frame, rollout=-1):
        if rollout == -1:
            rollout = self.rollout
        self.c.execute(update_score_sql, (score, frame, rollout)) 

    def update_reward(self, reward, frame, rollout=-1):
        if rollout == -1:
            rollout = self.rollout
        self.c.execute(update_reward_sql, (reward, frame, rollout)) 

    def dump_db(self, where):
        self.c.execute(dump_sql)
        rows = self.c.fetchall()
        print where
        for row in rows:
            print row
    
    def insert_rollout_rec(self):
        self.c.execute(insert_rollout_rec_sql, (self.rollout, ))

    def delete_rollout_rec(self, r):
        self.c.execute(delete_rollout_rec_sql, (r,))

    def update_rollout_rec_reward(self, r):
        self.c.execute(update_rollout_rec_reward_sql, (r,))

    def update_rollout_rec_used(self):
        self.c.execute(update_rollout_rec_used_sql)

    def list_unrewarded_rollouts(self):
        self.c.execute(list_unrewarded_rollouts_sql)
        return self.c.fetchall()

    def list_used_rollouts(self, state):
        self.c.execute(list_used_rollouts_sql, (state,))
        return self.c.fetchall()

if __name__ == '__main__':
    # A bit of test code
    my_db = db('tst.db', True) 
    for i in range(6):
        my_db.save_record(i, "file1", 1)
    for i in range(6):
        my_db.save_record(i, "file2", 2)
    for i in range(6):
        my_db.save_record(i, "file3", 3)
    for i in range(6):
        my_db.save_record(i, "file4", 4)
    my_db.commit()
    my_db.dump_db("End of test")
