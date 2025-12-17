package com.example.mirna.ui.home

import android.graphics.Color
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import com.example.mirna.databinding.FragmentHomeBinding
import okhttp3.*
import org.json.JSONObject
import java.io.IOException

class HomeFragment : Fragment() {

    private var _binding: FragmentHomeBinding? = null
    private val binding get() = _binding!!

    private val client = OkHttpClient()
    private val handler = Handler(Looper.getMainLooper())
    private lateinit var envRunnable: Runnable

    // Replace with your PC IP
    private val pcEnvUrl = "http://10.0.2.2:5000/env"

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        _binding = FragmentHomeBinding.inflate(inflater, container, false)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        startEnvPolling()
    }

    private fun fetchEnvData() {
        val request = Request.Builder().url(pcEnvUrl).build()

        client.newCall(request).enqueue(object : Callback {

            override fun onFailure(call: Call, e: IOException) {
                activity?.runOnUiThread {
                    binding.temperatureWarning.text = "Failed to fetch sensor data"
                    binding.temperatureWarning.setTextColor(Color.RED)
                }
            }

            override fun onResponse(call: Call, response: Response) {
                val body = response.body?.string() ?: return
                val json = JSONObject(body)

                val temp = json.optDouble("temperature", Double.NaN)
                val hum = json.optDouble("humidity", Double.NaN)

                activity?.runOnUiThread {
                    if (_binding == null) return@runOnUiThread  // ← SAFETY CHECK

                    if (temp.isNaN() || hum.isNaN()) {
                        binding.temperatureWarning.text = "Waiting for sensor data..."
                        binding.temperatureWarning.setTextColor(Color.GRAY)
                        return@runOnUiThread
                    }

                    val threshold = 30.0

                    binding.temperatureWarning.text =
                        "Room: ${"%.1f".format(temp)}°C, Humidity: ${"%.1f".format(hum)}%"

                    binding.temperatureWarning.setTextColor(
                        if (temp > threshold)
                            Color.parseColor("#B00020") // red
                        else
                            Color.parseColor("#006400") // green
                    )
                }
            }
        })
    }

    private fun startEnvPolling() {
        envRunnable = object : Runnable {
            override fun run() {
                fetchEnvData()
                handler.postDelayed(this, 2000)
            }
        }
        handler.post(envRunnable)
    }

    override fun onDestroyView() {
        super.onDestroyView()

        // ❗ Prevent crash when switching fragments
        handler.removeCallbacks(envRunnable)

        _binding = null
    }
}
