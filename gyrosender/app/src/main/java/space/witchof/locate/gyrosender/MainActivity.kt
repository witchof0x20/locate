package space.witchof.locate.gyrosender

import android.content.Context
import android.hardware.Sensor
import android.hardware.Sensor.TYPE_GYROSCOPE
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.os.PowerManager
import android.os.PowerManager.WakeLock
import android.os.StrictMode
import android.os.StrictMode.ThreadPolicy
import android.util.Log
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import space.witchof.locate.gyrosender.ui.theme.GyroSenderTheme
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.nio.ByteBuffer


class MainActivity : ComponentActivity(), SensorEventListener {
    private lateinit var mSensorManager:SensorManager
    //private lateinit var mAccelerometer :Sensor
    private lateinit var mGyroscope :Sensor
    private lateinit var ipAddr: InetAddress;
    private lateinit var clientSocket: DatagramSocket;
    private lateinit var wl: WakeLock;
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        mSensorManager = getSystemService(SENSOR_SERVICE) as SensorManager;
        //mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)!!
        mGyroscope = mSensorManager.getDefaultSensor(TYPE_GYROSCOPE)!!
        clientSocket = DatagramSocket()
        ipAddr = InetAddress.getByName("192.168.178.1")
        val pm: PowerManager = getSystemService(Context.POWER_SERVICE) as PowerManager;
        this.wl = pm.newWakeLock(PowerManager.SCREEN_DIM_WAKE_LOCK, "myapp:mytag");
        val gfgPolicy = ThreadPolicy.Builder().permitAll().build()
        StrictMode.setThreadPolicy(gfgPolicy)
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContent {
            GyroSenderTheme {
                // A surface container using the 'background' color from the theme
                Surface(
                    modifier = Modifier.fillMaxSize(),
                    color = MaterialTheme.colorScheme.background
                ) {
                    Greeting("Android")
                }
            }
        }
    }
    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {

    }
    override fun onSensorChanged(event: SensorEvent?) {
        wl.acquire()
        val sensorName: String = event?.sensor!!.getName();
        Log.i("Sensor",sensorName + ": X: " + event.values[0] + "; Y: " + event.values[1] + "; Z: " + event.values[2] + ";")
        var buffer: ByteBuffer  = ByteBuffer.allocate(18);
        buffer.putShort(5555).putFloat(0.0f).putFloat(event.values[0]).putFloat(event.values[1]).putFloat(event.values[2])
        buffer.rewind()
        val byteArray = ByteArray(18)
        buffer.get(byteArray)
        val send_packet = DatagramPacket(byteArray, 18, ipAddr, 5555)
        clientSocket = DatagramSocket()
        clientSocket.send(send_packet);
        wl.release();
    }
    override fun onResume() {
        super.onResume()
        //mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL)
        mSensorManager.registerListener(this, mGyroscope, SensorManager.SENSOR_DELAY_NORMAL)
    }

    override fun onPause() {
        super.onPause()
        mSensorManager.unregisterListener(this)
    }
}

@Composable
fun Greeting(name: String, modifier: Modifier = Modifier) {
    Text(
        text = "Hello $name!",
        modifier = modifier
    )
}

@Preview(showBackground = true)
@Composable
fun GreetingPreview() {
    GyroSenderTheme {
        Greeting("Android")
    }
}

