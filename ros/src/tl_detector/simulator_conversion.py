import tensorflow as tf
from object_detection.utils import dataset_util
from glob import glob
from sklearn.model_selection import train_test_split

flags = tf.app.flags
flags.DEFINE_string('train_path', 'train.record', 'Path to Train TFRecord')
flags.DEFINE_string('test_path', 'test.record', 'Path to Test TFRecord')
FLAGS = flags.FLAGS



height = 600 # Image height
width = 800 # Image width

def process_one(img_fpath, label_fpath, png_placeholder, sess):
    filename = str.encode(img_fpath)
    image_format = 'JPEG'.encode()

    with tf.gfile.GFile(img_fpath, 'rb') as fid:
          img = fid.read() # Encoded image bytes

    decoded_png = tf.image.decode_png(png_placeholder, channels=3)
    encode_op = tf.image.encode_jpeg(decoded_png, format='rgb', quality=100)

    encoded_image_data = sess.run(encode_op, feed_dict={png_placeholder: img})

    bboxs = []
    labels = []
    text_labels = []
    f = open(label_fpath, "r")
    for line in f:
        line = line.split()
        temp = []
        if len(line) != 1:
            for elem in line:
                temp.append(float(elem))
            bboxs.append(temp)
            labels.append(1)
            text_labels.append('Light'.encode())

    xmins = []
    xmaxs = []
    ymins = []
    ymaxs = []
    for box in bboxs:
        xmins.append(box[0] / width)
        xmaxs.append(box[2] / width)
        ymins.append(box[1] / height)
        ymaxs.append(box[3] / height)

    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(filename),
        'image/source_id': dataset_util.bytes_feature(filename),
        'image/encoded': dataset_util.bytes_feature(encoded_image_data),
        'image/format': dataset_util.bytes_feature(image_format),
        'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
        'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
        'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
        'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
        'image/object/class/text': dataset_util.bytes_list_feature(text_labels),
        'image/object/class/label': dataset_util.int64_list_feature(labels),
    }))
    return tf_example



def process_all(dict_to_process, output, png_placeholder, sess):
    writer = tf.python_io.TFRecordWriter(output)
    img_fpaths = dict_to_process['img']
    label_fpaths = dict_to_process['label']

    assert len(img_fpaths) == len(label_fpaths)
    for i in range(len(img_fpaths)):
        img_fpath = img_fpaths[i]
        label_fpath = label_fpaths[i]
        record = process_one(img_fpath, label_fpath, png_placeholder, sess)
        writer.write(record.SerializeToString())

    print ("TFRecords has been generated successfully")
    writer.close()

def find_and_split():
    Gimg_list = glob('Images/001/*.png')
    Rimg_list = glob('Images/002/*.png')
    Yimg_list = glob('Images/003/*.png')
    Gimg_list.sort()
    Rimg_list.sort()
    Yimg_list.sort()

    Glabel_list = glob('Labels/001/*.txt')
    Rlabel_list = glob('Labels/002/*.txt')
    Ylabel_list = glob('Labels/003/*.txt')
    Glabel_list.sort()
    Rlabel_list.sort()
    Ylabel_list.sort()

    assert len(Gimg_list) == len(Glabel_list)
    assert len(Rimg_list) == len(Rlabel_list)
    assert len(Yimg_list) == len(Ylabel_list)

    X = Gimg_list + Rimg_list + Yimg_list
    Y = Glabel_list + Rlabel_list + Ylabel_list

    split = .75
    Xtrain, Xtest, Ytrain, Ytest = train_test_split(X, Y, train_size=split, test_size=1-split)

    train = {'img': Xtrain, 'label': Ytrain}
    test = {'img': Xtest, 'label': Ytest}

    return train, test


def main(_):
    train, test = find_and_split()

    png_placeholder = tf.placeholder(dtype=tf.string)
    with tf.Session() as sess:
        process_all(train, FLAGS.train_path, png_placeholder, sess)
        process_all(test, FLAGS.test_path, png_placeholder, sess)

if __name__ == '__main__':
  tf.app.run()
