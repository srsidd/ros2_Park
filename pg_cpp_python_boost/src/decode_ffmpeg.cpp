// This file creates a standalone H.264 stream decoder using FFMPEG
// which can be imported into a Python script. The stream format has
// been extended with a timestamp as the first 8 bytes of each packet.

#include <iostream>
#include <thread>
// #include <string.h>
#include <vector>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
using namespace boost::python;

extern "C"
{
#include <libswscale/swscale.h>
#include <libavcodec/avcodec.h>
}

struct FFMPEG_decoder {
    AVCodec* decoder = nullptr;
    AVCodecContext* decoderContext = nullptr;
    int got_picture = 0;
    AVFrame* picture = nullptr;
    AVPacket packet;
    int out_width, out_height;

    int64_t prev_timestamp = 0;

    FFMPEG_decoder(int w, int h, enum AVCodecID type);
    ~FFMPEG_decoder();
    void decode_frame(uint8_t* data, int size, void (*callback)(uint8_t*, int, int64_t));
    void decode_frame_python(int64_t timestamp, std::vector<uint8_t>& data_in, std::vector<uint8_t> &data_out);
};


FFMPEG_decoder::FFMPEG_decoder(int w, int h, enum AVCodecID type) : out_width(w), out_height(h) {
    avcodec_register_all();
    av_init_packet(&packet);
    decoder = avcodec_find_decoder(type);
    if (!decoder) {
        std::cout << "Error: H.264 encoder not found\n" << std::endl;
    }
    decoderContext = avcodec_alloc_context3(decoder);

    if (decoder->capabilities & CODEC_CAP_TRUNCATED)
        decoderContext->flags |= CODEC_FLAG_TRUNCATED;

    // we can receive truncated frames
    decoderContext->flags2 |= CODEC_FLAG2_CHUNKS;
    // get the number of threads available on the system
    decoderContext->thread_count = std::thread::hardware_concurrency();

    if (avcodec_open2(decoderContext, decoder, nullptr) < 0) {
        std::cout << "Error: avcodec_open2 failed\n";
        return;
    }

#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(55, 45, 101)
    picture = av_frame_alloc();
#else
    picture = avcodec_alloc_frame();
#endif
}

FFMPEG_decoder::~FFMPEG_decoder() {
    if (decoderContext)
    {
        avcodec_close(decoderContext);
        delete decoderContext;
    }
}

// warning: the callback must make a copy of the frame before it is deleted
void FFMPEG_decoder::decode_frame(uint8_t* data, int size, void (*callback)(uint8_t*, int, int64_t)) {
    if(callback == nullptr) {
        std::cout << "error: FFMPEG_decoder::decode_frame callback cannot be nullptr\n";
        return;
    }
    std::cout << "decode_frame size: " << size << '\n';
    packet.size = size;
    packet.data = data;
    int size_out;
    int read_bytes = 0;
    int64_t timestamp;
    memcpy(&timestamp, data, sizeof(timestamp));

    while(packet.size > 0)
    {
        int got_picture;
        int len = avcodec_decode_video2(decoderContext, picture, &got_picture, &packet);
        if (len < 0) {
            std::cout << "error: avcodec_decode_video2 failed to decode input\n";
            size_out = -1;
            return;
        }
        if (got_picture)
        {
            if ( 1 || picture->format == AV_PIX_FMT_YUV422P )
            {
                static SwsContext *swsCtx = NULL;
                swsCtx = sws_getCachedContext ( swsCtx, picture->width,
                    picture->height, (AVPixelFormat)picture->format,
                    out_width, out_height,
                    AV_PIX_FMT_RGB24, SWS_BICUBIC,
                    NULL, NULL, NULL );
                size_out = out_width*out_height*3;
                uint8_t *dstSlice[] = { new uint8_t[size_out] };
                int dstStride = out_width * 3;
                sws_scale ( swsCtx, picture->data, picture->linesize, 0, out_height, dstSlice, &dstStride);
                callback(dstSlice[0], size_out, timestamp);
                delete dstSlice[0];
                prev_timestamp = timestamp;
            }
            else {
                std::cout << "unsupported color format from avcodec_decode_video2 " << picture->format << "\n";
                size_out = -1;
                return;
            }
        }
        packet.size -= len;
        packet.data += len;
        read_bytes += len;
    }
}


void FFMPEG_decoder::decode_frame_python(int64_t timestamp, std::vector<uint8_t>& data_in, std::vector<uint8_t>& data_out)
{
    std::cout << "decode_frame_python called" << std::endl;
    std::cout << "  data_in len: " << data_in.size() << std::endl;
    std::cout << "  data_out len: " << data_out.size() << std::endl;

    //std::cout << "decode_frame size: " << size << '\n';
    packet.size = data_in.size();
    packet.data = data_in.data();
    int size_out;
    int read_bytes = 0;
    // int64_t timestamp;
    // memcpy(&timestamp, data, sizeof(timestamp));

    while(packet.size > 0)
    {
        int got_picture;
        int len = avcodec_decode_video2(decoderContext, picture, &got_picture, &packet);
        if (len < 0) {
            std::cout << "error: avcodec_decode_video2 failed to decode input\n";
            size_out = -1;
            return;
        }
        if (got_picture)
        {
            if ( 1 || picture->format == AV_PIX_FMT_YUV422P )
            {
                static SwsContext *swsCtx = NULL;
                swsCtx = sws_getCachedContext ( swsCtx, picture->width,
                    picture->height, (AVPixelFormat)picture->format,
                    out_width, out_height,
                    AV_PIX_FMT_RGB24, SWS_BICUBIC,
                    NULL, NULL, NULL );
                size_out = out_width*out_height*3;
                // uint8_t *dstSlice[] = { new uint8_t[size_out] };
                uint8_t* dstSlice = data_out.data();
                data_out.resize(size_out);
                int dstStride = out_width * 3;
                // sws_scale ( swsCtx, picture->data, picture->linesize, 0, out_height, dstSlice, &dstStride);
                sws_scale ( swsCtx, picture->data, picture->linesize, 0, out_height, &dstSlice, &dstStride);
                // callable(dstSlice[0], size_out, timestamp);
                // delete dstSlice[0];
                prev_timestamp = timestamp;
            }
            else {
                std::cout << "unsupported color format from avcodec_decode_video2 " << picture->format << "\n";
                size_out = -1;
                return;
            }
        }
        packet.size -= len;
        packet.data += len;
        read_bytes += len;
    }
}

// extern "C" {
// void* initFFMPEG_H264_decoder(int width, int height) {
//     return new FFMPEG_decoder(width, height, AV_CODEC_ID_H264);
// }
//
// void* initFFMPEG_MJPG_decoder(int width, int height) {
//     return new FFMPEG_decoder(width, height, AV_CODEC_ID_MJPEG);
// }
//
// void decode_frame(void* dec, uint8_t* data, int size, void (*callback)(uint8_t*, int, int64_t)) {
//     ((FFMPEG_decoder*)dec)->decode_frame(data, size, callback);
// }
//
// void deleteFFMPEG_decoder(FFMPEG_decoder* dec) {
//     delete dec;
// }
// }

void py_decode_frame(FFMPEG_decoder& self, long timestamp, boost::python::object py_buf_in, boost::python::object py_buf_out)
{
    std::cout << "hello world!" << std::endl;
    // `str` objects do not implement the iterator protcol (__iter__),
    // but do implement the sequence protocol (__getitem__).  Use the
    // `iter()` builtin to create an iterator for the buffer.
    // >>> __builtins__.iter(py_buffer)
    boost::python::object locals(boost::python::borrowed(PyEval_GetLocals()));
    std::cout << "hello world! 1" << std::endl;
    boost::python::object py_iter = locals["__builtins__"].attr("iter");
    std::cout << "hello world! 2" << std::endl;
    boost::python::stl_input_iterator<uint8_t> begin_in(py_iter(py_buf_in)), begin_out(py_iter(py_buf_out)), end;

    // Copy the py_buffer into a local buffer with known continguous memory.
    std::cout << "hello world! 3" << std::endl;
    std::vector<uint8_t> buffer_in(begin_in, end);
    std::cout << "hello world! 4" << std::endl;
    std::vector<uint8_t> buffer_out(begin_out, end);

    // self.decode_frame_python(timestamp, buffer_in, buffer_out)
}


BOOST_PYTHON_MODULE(libdecode)
{
    enum_<AVCodecID>("codec")
        .value("H264", AV_CODEC_ID_H264)
        .value("MJPEG", AV_CODEC_ID_MJPEG);

    class_<FFMPEG_decoder>("VideoDecoder", init<int, int, enum AVCodecID>())
        .def("decode_frame", &py_decode_frame);
}
