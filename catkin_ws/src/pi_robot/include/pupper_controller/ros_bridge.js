const ROSLIB = require('roslib')

var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
})

ros.on('connection', function () {
    console.log('Connected to localhost.')
})

ros.on('error', function (error) {
    console.log('Error connecting to localhost: ', error)
})

ros.on('close', function () {
    console.log('Connection to localhost closed.')
})

// Publishing a Topic
// ------------------

var cmdJointState = new ROSLIB.Topic({
    ros: ros,
    name: '/joint_states',
    messageType: 'sensor_msgs/JointState'
})


function connectToMaster(ip) {

    // Connecting to ROS
    // -----------------

    var ros = new ROSLIB.Ros({
        url: `ws://${ip}:9090`
    })

    ros.on('connection', function () {
        console.log('Connected to websocket server.')
    })

    ros.on('error', function (error) {
        console.log('Error connecting to websocket server: ', error)
    })

    ros.on('close', function () {
        console.log('Connection to websocket server closed.')
    })

    // Subscribing to a Topic
    // ----------------------

    var listener = new ROSLIB.Topic({
        ros: ros,
        name: '/joint_states',
        messageType: 'sensor_msgs/JointState'
    })

    listener.subscribe(function (message) {
        // console.log('Received message on ' + listener.name + ': ', message)
        cmdJointState.publish(message)
        // listener.unsubscribe()
    })
}
if (process.argv.length < 3) {
    console.log('usage: node xxxxx.js MASTER_IP')
} else {
    connectToMaster(process.argv[2])
}