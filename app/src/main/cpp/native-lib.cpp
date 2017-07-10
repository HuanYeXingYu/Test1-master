#include <jni.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <stdint.h>
#include <termios.h>
#include <android/log.h>
#include <sys/ioctl.h>

extern "C" {


JNIEXPORT jstring JNICALL
Java_com_example_test1_Serial_Serial_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

//串口通信

int fd = 0;

/*
 * Class:     com_topeet_serialtest_serial
 * Method:    Open
 * Signature: ()I
 */
JNIEXPORT jint Java_com_example_test1_Serial_Serial_Open
        (JNIEnv *env, jobject obj, jint Port, jint Rate) {
    if (fd <= 0) {
        if (0 == Port) {
            __android_log_print(ANDROID_LOG_INFO, "serial", "open fd /dev/ttySAC0");
            fd = open("/dev/ttySAC0", O_RDWR | O_NDELAY | O_NOCTTY);
        } else if (1 == Port) {
            __android_log_print(ANDROID_LOG_INFO, "serial", "open fd /dev/ttySAC1");
            fd = open("/dev/ttySAC1", O_RDWR | O_NDELAY | O_NOCTTY);
        } else if (2 == Port) {
            __android_log_print(ANDROID_LOG_INFO, "serial", "open fd /dev/ttySAC2");
            fd = open("/dev/ttySAC2", O_RDWR | O_NDELAY | O_NOCTTY);
        } else if (3 == Port) {
            __android_log_print(ANDROID_LOG_INFO, "serial", "open fd /dev/ttySAC3");
            fd = open("/dev/ttySAC3", O_RDWR | O_NDELAY | O_NOCTTY);
        } else if (4 == Port) {
            __android_log_print(ANDROID_LOG_INFO, "serial", "open fd /dev/ttyUSB0");
            fd = open("/dev/ttyUSB0", O_RDWR | O_NDELAY | O_NOCTTY);
        } else if (5 == Port) {
            __android_log_print(ANDROID_LOG_INFO, "serial", "open fd /dev/ttyUSB1");
            fd = open("/dev/ttyUSB1", O_RDWR | O_NDELAY | O_NOCTTY);
        } else {
            __android_log_print(ANDROID_LOG_INFO, "serial", "Parameter Error serial not found");
            fd = 0;
            return -1;
        }
#if 1
        if (fd > 0) {
            __android_log_print(ANDROID_LOG_INFO, "serial", "serial open ok fd=%d", fd);
            // disable echo on serial ports
            struct termios ios;
            tcgetattr(fd, &ios);
            ios.c_oflag &= ~(INLCR | IGNCR | ICRNL);
            ios.c_oflag &= ~(ONLCR | OCRNL);
            ios.c_iflag &= ~(ICRNL | IXON);
            ios.c_iflag &= ~(INLCR | IGNCR | ICRNL);
            ios.c_iflag &= ~(ONLCR | OCRNL);
            tcflush(fd, TCIFLUSH);

            if (Rate == 2400) {
                cfsetospeed(&ios, B2400);
                cfsetispeed(&ios, B2400);
            }
            if (Rate == 4800) {
                cfsetospeed(&ios, B4800);
                cfsetispeed(&ios, B4800);
            }
            if (Rate == 9600) {
                cfsetospeed(&ios, B9600);
                cfsetispeed(&ios, B9600);
            }
            if (Rate == 19200) {
                cfsetospeed(&ios, B19200);
                cfsetispeed(&ios, B19200);
            }
            if (Rate == 38400) {
                cfsetospeed(&ios, B38400);
                cfsetispeed(&ios, B38400);
            }
            if (Rate == 57600) {
                cfsetospeed(&ios, B57600);
                cfsetispeed(&ios, B57600);
            }
            if (Rate == 115200) {
                cfsetospeed(&ios, B115200);
                cfsetispeed(&ios, B115200);
            }

            ios.c_cflag |= (CLOCAL | CREAD);
            ios.c_cflag &= ~PARENB;
            ios.c_cflag &= ~CSTOPB;
            ios.c_cflag &= ~CSIZE;
            ios.c_cflag |= CS8;
            ios.c_lflag = 0;
            tcsetattr(fd, TCSANOW, &ios);
        }
#endif
    }

    return fd;
}

/*
 * Class:     com_topeet_serialtest_serial
 * Method:    Close
 * Signature: ()I
 */
JNIEXPORT jint Java_com_example_test1_Serial_Serial_Close
        (JNIEnv *env, jobject obj) {
    if (fd > 0)close(fd);
}


/*
 * Class:     com_topeet_serialtest_serial
 * Method:    Read
 * Signature: ()[I
 */
JNIEXPORT jintArray Java_com_example_test1_Serial_Serial_Read
        (JNIEnv *env, jobject obj) {
    unsigned char buffer[1024];
    int BufToJava[1024];
    int len = 0, i = 0;

    memset(buffer, 0, sizeof(buffer));
    memset(BufToJava, 0, sizeof(BufToJava));

    len = read(fd, buffer, 1024);
//read()会把参数fd 所指的文件传送count 个字节到buf 指针所指的内存中.
//若参数count 为0, 则read()不会有作用并返回0. 返回值为实际读取到的字节数,
//如果返回0, 表示已到达文件尾或是无可读取的数据,此外文件读写位置会随读取到的字节移动.
    if (len <= 0)return NULL;

    for (i = 0; i < len; i++) {
        printf("%x", buffer[i]);
        BufToJava[i] = buffer[i];
    }

    jintArray array = env->NewIntArray(len);
    env->SetIntArrayRegion(array, 0, len, BufToJava);

    return array;
}

/*
 * Class:     com_topeet_serialtest_serial
 * Method:    Read
 * Signature: ()[I
 */
JNIEXPORT jint Java_com_example_test1_Serial_Serial_Write
        (JNIEnv *env, jobject obj, jintArray buf, jint buflen) {
    jsize len = buflen;

    if (len <= 0)
        return -1;

    jintArray array = env->NewIntArray(len);

    if (array == NULL) {
        array = NULL;
        return -1;
    }

    jint *body = env->GetIntArrayElements(buf, 0);

    jint i = 0;
    unsigned char num[len];

    for (; i < len; i++)
        num[i] = body[i];

    write(fd, num, len);

    array = NULL;

    return 0;
}


static const char *TAG = "serial_port";
#define LOGI(fmt, args...) __android_log_print(ANDROID_LOG_INFO,  TAG, fmt, ##args)
#define LOGD(fmt, args...) __android_log_print(ANDROID_LOG_DEBUG, TAG, fmt, ##args)
#define LOGE(fmt, args...) __android_log_print(ANDROID_LOG_ERROR, TAG, fmt, ##args)

static speed_t getBaudrate(jint baudrate) {
    switch (baudrate) {
        case 0:
            return B0;
        case 50:
            return B50;
        case 75:
            return B75;
        case 110:
            return B110;
        case 134:
            return B134;
        case 150:
            return B150;
        case 200:
            return B200;
        case 300:
            return B300;
        case 600:
            return B600;
        case 1200:
            return B1200;
        case 1800:
            return B1800;
        case 2400:
            return B2400;
        case 4800:
            return B4800;
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        case 1152000:
            return B1152000;
        case 1500000:
            return B1500000;
        case 2000000:
            return B2000000;
        case 2500000:
            return B2500000;
        case 3000000:
            return B3000000;
        case 3500000:
            return B3500000;
        case 4000000:
            return B4000000;
        default:
            return -1;
    }
}

/*
 * Class:     android_serialport_SerialPort
 * Method:    open
 * Signature: (Ljava/lang/String;II)Ljava/io/FileDescriptor;
 */
JNIEXPORT jobject JNICALL Java_com_example_test1_Serial_SerialPort_open
        (JNIEnv *env, jobject thiz, jstring path, jint baudrate)
{
    int fd;
    speed_t speed;
    jobject mFileDescriptor;

    /* Check arguments */
    {
        speed = getBaudrate(baudrate);
        if (speed == -1) {
            /* TODO: throw an exception */
            LOGE("Invalid baudrate");
            return NULL;
        }
    }

    /* Opening device */
    {
        jboolean iscopy;
        const char *path_utf = env->GetStringUTFChars( path, &iscopy);
        LOGD("Opening serial port %s", path_utf);
        fd = open(path_utf, O_RDWR | O_DIRECT | O_SYNC);
        LOGD("open() fd = %d", fd);
        env->ReleaseStringUTFChars( path, path_utf);
        if (fd == -1)
        {
            /* Throw an exception */
            LOGE("Cannot open port");
            /* TODO: throw an exception */
            return NULL;
        }
    }

    /* Configure device */
    {
        struct termios cfg;
        LOGD("Configuring serial port");
        if (tcgetattr(fd, &cfg))
        {
            LOGE("tcgetattr() failed");
            close(fd);
            /* TODO: throw an exception */
            return NULL;
        }

        cfmakeraw(&cfg);
        cfsetispeed(&cfg, speed);
        cfsetospeed(&cfg, speed);

        if (tcsetattr(fd, TCSANOW, &cfg))
        {
            LOGE("tcsetattr() failed");
            close(fd);
            /* TODO: throw an exception */
            return NULL;
        }
    }

    /* Create a corresponding file descriptor */
    {
        jclass cFileDescriptor = env->FindClass( "java/io/FileDescriptor");
        jmethodID iFileDescriptor = env->GetMethodID( cFileDescriptor, "<init>", "()V");
        jfieldID descriptorID = env->GetFieldID( cFileDescriptor, "descriptor", "I");
        mFileDescriptor = env->NewObject( cFileDescriptor, iFileDescriptor);
        env->SetIntField( mFileDescriptor, descriptorID, (jint)fd);
    }

    return mFileDescriptor;
}

/*
 * Class:     cedric_serial_SerialPort
 * Method:    close
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_example_test1_Serial_SerialPort_close
        (JNIEnv *env, jobject thiz) {
    jclass SerialPortClass = env->GetObjectClass(thiz);
    jclass FileDescriptorClass = env->FindClass("java/io/FileDescriptor");

    jfieldID mFdID = env->GetFieldID(SerialPortClass, "mFd", "Ljava/io/FileDescriptor;");
    jfieldID descriptorID = env->GetFieldID(FileDescriptorClass, "descriptor", "I");

    jobject mFd = env->GetObjectField(thiz, mFdID);
    jint descriptor = env->GetIntField(mFd, descriptorID);

    LOGD("close(fd = %d)", descriptor);
    close(descriptor);
}


}
