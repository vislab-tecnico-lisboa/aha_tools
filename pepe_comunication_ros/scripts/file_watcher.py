import pyinotify
from datetime import datetime
import pandas as pd
import requests
import json
import argparse

class ModHandler(pyinotify.ProcessEvent):
    # evt has useful properties, including pathname
    def __init__(self,filename):
        super(ModHandler,self).__init__()
        self.filename = filename
        self.old_df = pd.read_csv(filename)
    def process_IN_MODIFY(self, evt):
            new_df = pd.read_csv(self.filename)
            diff = pd.concat([self.old_df,new_df]).drop_duplicates(keep=False)
            r = requests.post("http://127.0.0.1:5000", data=json.dumps({'diff': str(diff.values)}))
            self.old_df = new_df.copy()
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='What File')
    parser.add_argument('filename',type=str,
                         help='file name to watch')
    """
    parser.add_argument('post_endpoint',
                        type = str,
                        help='end point to publish the differences')
    """
    args = parser.parse_args()
    handler = ModHandler(args.filename)
    wm = pyinotify.WatchManager()
    notifier = pyinotify.Notifier(wm, handler)
    wdd = wm.add_watch(args.filename, pyinotify.IN_MODIFY)
    notifier.loop()
