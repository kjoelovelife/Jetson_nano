#!/usr/bin/env python
from duckietown_utils.bag_logs import d8n_get_all_images_topic
from procgraph.scripts.pgmain import pg
from quickapp import QuickApp  # @UnresolvedImport
import logging
import os
import shutil
logging.basicConfig()
logger = logging.getLogger(__name__)
from decent_params.utils import UserError

class MakeVideo(QuickApp):
    """ Simplest app example """

    def define_options(self, params):
        params.add_string('dir', help='Directory containing logs')
        params.add_string('tmpdir', help='Working directory ', default='tmp')

    def define_jobs_context(self, context):
        options = self.get_options()
        dirname = options.dir
        if not os.path.exists(dirname):
            msg = "Path does not exist: %r." % dirname
            raise UserError(msg)

        from conf_tools.utils import locate_files  # @UnresolvedImport
        bags = list(locate_files(dirname, pattern="*.bag", followlinks=True))
        self.info('I found %d bags in %s' % (len(bags), dirname))

        if len(bags) == 0:
            msg = "Could not find any bag in %r." % dirname
            raise UserError(msg)

        def short(f):
            return os.path.splitext(os.path.basename(f))[0]

        for f in bags:
            s = short(f)
            context.comp_dynamic(process, bag_filename=f,
                                 tmpdir=options.tmpdir,
                                 models=['bag2mp4_fixfps'], job_id=s)


def process(context, bag_filename, tmpdir, models=['bag2mp4_fixfps', 'bag2mp4']):
    res = d8n_get_all_images_topic(bag_filename)

    for topic, msg_type in res:

        sanitized = topic.replace('/', '_')
        sanitized = sanitized[1:]  # remove first
        for model in models:
            bn = os.path.splitext(os.path.basename(bag_filename))[0]
            bn = bn + '-%s-%s.mp4' % (model, sanitized)
            dn = os.path.dirname(bag_filename)
            where = '_v1'
            out = os.path.join(dn, where, bn)

            context.comp(visualize_topic, bag_filename=bag_filename,
                         model=model,
                         topic=topic,
                         out=out, tmpdir=tmpdir)
        

def visualize_topic(bag_filename, model, topic, out, tmpdir):
    # pg -m procgraph_ros bag2mp4 --bag $bag --topic $topic --out $out
    import procgraph_ros  # @UnusedImport
    out_tmp = os.path.join(tmpdir, os.path.basename(out))
    print('writing to %r' % out)
    pg(model, config=dict(bag=bag_filename, topic=topic, out=out_tmp))
    md = out_tmp + '.metadata.yaml'
    if os.path.exists(md):
        os.unlink(md)
    # copy out_tmp to out
    if True: # only AC
        dn = os.path.dirname(out)
        if not os.path.exists(dn):
            os.makedirs(dn)
        print('copying to %r' % out)
        shutil.copyfile(out_tmp, out)

    info = bag_filename + '.info.yaml'
    if os.path.exists(info):
        os.unlink(info)


if __name__ == '__main__':
    app_example_main = MakeVideo.get_sys_main()
    app_example_main()

