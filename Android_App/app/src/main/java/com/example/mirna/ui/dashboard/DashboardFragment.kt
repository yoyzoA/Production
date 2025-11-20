package com.example.mirna.ui.dashboard

import android.view.View
import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.media.MediaMuxer
import android.media.MediaScannerConnection
import android.os.Bundle
import android.os.Environment
import android.webkit.WebViewClient
import androidx.fragment.app.Fragment
import com.example.mirna.databinding.FragmentDashboardBinding
import kotlinx.coroutines.*
import java.io.ByteArrayOutputStream
import java.io.File
import java.net.HttpURLConnection
import java.net.URL
import android.graphics.Bitmap
import android.graphics.BitmapFactory

class DashboardFragment : Fragment() {

    private var _binding: FragmentDashboardBinding? = null
    private val binding get() = _binding!!

    // MJPEG stream
    private val streamUrl = "http://10.0.2.2:5000/video" // emulator → host

    // Recording
    private var recordingJob: Job? = null
    @Volatile
    private var isRecording = false

    // Encoding params (fast)
    private val VIDEO_WIDTH = 640
    private val VIDEO_HEIGHT = 480
    private val VIDEO_FPS = 15
    private val VIDEO_BITRATE = 1_500_000 // 1.5 Mbps

    override fun onCreateView(
        inflater: android.view.LayoutInflater,
        container: android.view.ViewGroup?,
        savedInstanceState: Bundle?
    ): android.view.View {
        _binding = FragmentDashboardBinding.inflate(inflater, container, false)
        return binding.root
    }

    override fun onViewCreated(view: android.view.View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        setupWebView()

        binding.btnRecord.setOnClickListener {
            if (recordingJob == null) {
                startRecording()
            }
        }

        binding.btnStop.setOnClickListener {
            stopRecording()
        }
    }

    private fun setupWebView() {
        val webView = binding.webview
        webView.settings.javaScriptEnabled = true
        webView.settings.loadWithOverviewMode = true
        webView.settings.useWideViewPort = true
        webView.webViewClient = WebViewClient()
        webView.loadUrl(streamUrl)
    }

    private fun startRecording() {
        if (recordingJob != null) return

        isRecording = true

        // Show indicator
        requireActivity().runOnUiThread {
            val indicator = binding.recordIndicator
            indicator.visibility = View.VISIBLE
        }

        recordingJob = CoroutineScope(Dispatchers.IO).launch {
            recordMjpegToMp4()
        }
    }

    private fun stopRecording() {
        isRecording = false
        recordingJob?.cancel()
        recordingJob = null

        // Hide indicator
        requireActivity().runOnUiThread {
            binding.recordIndicator.clearAnimation()
            binding.recordIndicator.visibility = View.GONE
        }
    }

    private fun createMediaStoreVideoFile(): File {
        val path = "/storage/emulated/0/Movies/MyAppRecordings/"
        val folder = File(path)
        if (!folder.exists()) {
            folder.mkdirs()
        }
        val filename = "mirna_stream_${System.currentTimeMillis()}.mp4"
        return File(folder, filename)
    }


    /**
     * Open the MJPEG stream, extract JPEG frames, transcode to MP4 using MediaCodec + MediaMuxer
     */
    private fun recordMjpegToMp4() {
        var connection: HttpURLConnection? = null
        var codec: MediaCodec? = null
        var muxer: MediaMuxer? = null

        try {
            // --- Create output file in app's Movies dir ---
            val moviesDir = requireContext()
                .getExternalFilesDir(Environment.DIRECTORY_MOVIES) ?: requireContext().filesDir
            val outFile = createMediaStoreVideoFile()


            // --- Configure MediaCodec encoder ---
            val format = MediaFormat.createVideoFormat(
                MediaFormat.MIMETYPE_VIDEO_AVC,
                VIDEO_WIDTH,
                VIDEO_HEIGHT
            )
            format.setInteger(
                MediaFormat.KEY_COLOR_FORMAT,
                MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible
            )
            format.setInteger(MediaFormat.KEY_BIT_RATE, VIDEO_BITRATE)
            format.setInteger(MediaFormat.KEY_FRAME_RATE, VIDEO_FPS)
            format.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1)

            codec = MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_VIDEO_AVC)
            codec.configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
            codec.start()

            muxer = MediaMuxer(outFile.absolutePath, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4)
            var trackIndex = -1
            var muxerStarted = false
            val bufferInfo = MediaCodec.BufferInfo()

            // --- Open MJPEG stream ---
            val url = URL(streamUrl)
            connection = (url.openConnection() as HttpURLConnection).apply {
                connectTimeout = 5000
                readTimeout = 5000
                doInput = true
                connect()
            }

            val input = connection.inputStream

            val frameBuffer = ByteArrayOutputStream()
            val readBuffer = ByteArray(8 * 1024)

            var ptsUs: Long = 0
            val frameIntervalUs = 1_000_000L / VIDEO_FPS

            // We'll keep a simple MJPEG parser: look for JPEG SOI(FFD8) & EOI(FFD9)
            var bytesRead: Int=0
            var lastByte: Int = -1

            while (isRecording && input.read(readBuffer).also { bytesRead = it } != -1) {
                if (bytesRead <= 0) continue

                for (i in 0 until bytesRead) {
                    val b = readBuffer[i].toInt() and 0xFF
                    frameBuffer.write(b)

                    // detect end of JPEG: 0xFF,0xD9
                    if (lastByte == 0xFF && b == 0xD9) {
                        // we have one full JPEG in frameBuffer
                        val jpegBytes = frameBuffer.toByteArray()
                        frameBuffer.reset()

                        // Decode JPEG to bitmap
                        val bitmap = BitmapFactory.decodeByteArray(jpegBytes, 0, jpegBytes.size)
                            ?: continue

                        // Scale to our encoding resolution
                        val scaled = Bitmap.createScaledBitmap(
                            bitmap,
                            VIDEO_WIDTH,
                            VIDEO_HEIGHT,
                            true
                        )
                        bitmap.recycle()

                        // Convert to YUV (NV21-style)
                        val yuvData = bitmapToNV21(VIDEO_WIDTH, VIDEO_HEIGHT, scaled)
                        scaled.recycle()

                        // --- Feed frame into encoder ---
                        val inputIndex = codec.dequeueInputBuffer(10_000)
                        if (inputIndex >= 0) {
                            val inputBuffer = codec.getInputBuffer(inputIndex)
                            inputBuffer?.clear()
                            inputBuffer?.put(yuvData)
                            codec.queueInputBuffer(
                                inputIndex,
                                0,
                                yuvData.size,
                                ptsUs,
                                0
                            )
                            ptsUs += frameIntervalUs
                        }

                        // --- Drain encoder output ---
                        while (true) {
                            val outputIndex = codec.dequeueOutputBuffer(bufferInfo, 0)
                            when {
                                outputIndex == MediaCodec.INFO_TRY_AGAIN_LATER -> {
                                    break
                                }

                                outputIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED -> {
                                    if (muxerStarted) {
                                        throw RuntimeException("Format changed twice")
                                    }
                                    val newFormat = codec.outputFormat
                                    trackIndex = muxer.addTrack(newFormat)
                                    muxer.start()
                                    muxerStarted = true
                                }

                                outputIndex >= 0 -> {
                                    val encodedData = codec.getOutputBuffer(outputIndex)
                                        ?: continue

                                    if (bufferInfo.size != 0 && muxerStarted) {
                                        encodedData.position(bufferInfo.offset)
                                        encodedData.limit(bufferInfo.offset + bufferInfo.size)
                                        muxer.writeSampleData(trackIndex, encodedData, bufferInfo)
                                    }

                                    codec.releaseOutputBuffer(outputIndex, false)

                                    if (bufferInfo.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0) {
                                        break
                                    }
                                }
                            }
                        }
                    }

                    lastByte = b
                }
            }

            // signal EOS (end of stream)
            val inputIndex = codec.dequeueInputBuffer(10_000)
            if (inputIndex >= 0) {
                codec.queueInputBuffer(
                    inputIndex,
                    0,
                    0,
                    ptsUs,
                    MediaCodec.BUFFER_FLAG_END_OF_STREAM
                )
            }

            // drain remaining output
            while (true) {
                val outputIndex = codec.dequeueOutputBuffer(bufferInfo, 10_000)
                if (outputIndex == MediaCodec.INFO_TRY_AGAIN_LATER) break

                if (outputIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                    if (!muxerStarted) {
                        val newFormat = codec.outputFormat
                        trackIndex = muxer.addTrack(newFormat)
                        muxer.start()
                        muxerStarted = true
                    }
                } else if (outputIndex >= 0) {
                    val encodedData = codec.getOutputBuffer(outputIndex) ?: break
                    if (bufferInfo.size > 0 && muxerStarted) {
                        encodedData.position(bufferInfo.offset)
                        encodedData.limit(bufferInfo.offset + bufferInfo.size)
                        muxer.writeSampleData(trackIndex, encodedData, bufferInfo)
                    }
                    codec.releaseOutputBuffer(outputIndex, false)
                    if (bufferInfo.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0) {
                        break
                    }
                }
            }

            muxer.stop()
            muxer.release()
            muxer = null

            codec.stop()
            codec.release()
            codec = null

            // Make it visible in Gallery
            MediaScannerConnection.scanFile(
                requireContext(),
                arrayOf(outFile.absolutePath),
                arrayOf("video/mp4"),
                null
            )

        } catch (e: Exception) {
            e.printStackTrace()
        } finally {
            try {
                connection?.disconnect()
            } catch (_: Exception) {
            }
            try {
                codec?.stop()
                codec?.release()
            } catch (_: Exception) {
            }
            try {
                muxer?.stop()
                muxer?.release()
            } catch (_: Exception) {
            }
        }
    }

    /**
     * Convert ARGB Bitmap to NV21 (YUV420) byte array.
     * This is not super-optimized but fine for 640x480 @ ~15fps.
     */
    private fun bitmapToNV21(width: Int, height: Int, bitmap: Bitmap): ByteArray {
        val argb = IntArray(width * height)
        bitmap.getPixels(argb, 0, width, 0, 0, width, height)

        val yuv = ByteArray(width * height * 3 / 2)
        var yIndex = 0
        var uvIndex = width * height

        for (j in 0 until height) {
            for (i in 0 until width) {
                val c = argb[j * width + i]
                val r = (c shr 16) and 0xFF
                val g = (c shr 8) and 0xFF
                val b = c and 0xFF

                var y = (0.299 * r + 0.587 * g + 0.114 * b).toInt()
                var u = (-0.169 * r - 0.331 * g + 0.5 * b + 128).toInt()
                var v = (0.5 * r - 0.419 * g - 0.081 * b + 128).toInt()

                y = y.coerceIn(0, 255)
                u = u.coerceIn(0, 255)
                v = v.coerceIn(0, 255)

                yuv[yIndex++] = y.toByte()

                // 4:2:0 subsampling (NV21: VU interleaved)
                if (j % 2 == 0 && i % 2 == 0 && uvIndex + 1 < yuv.size) {
                    yuv[uvIndex++] = v.toByte()
                    yuv[uvIndex++] = u.toByte()
                }
            }
        }

        return yuv
    }

    override fun onDestroyView() {
        super.onDestroyView()
        stopRecording()
        _binding = null
    }
}
