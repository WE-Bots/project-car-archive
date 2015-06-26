var _speed = 500;
var _angle = 900;

//Called when application is started.
function OnStart() {
    //Create the frame for the diagnostic data to be shown on
    lay_diag = app.CreateLayout("linear");
    diag_text_voltage = app.CreateText("Voltage: ");
    diag_text_current = app.CreateText("Current: ");
    lay_diag.AddChild(diag_text_voltage);
    lay_diag.AddChild(diag_text_current);
    //Create a layout with objects vertically centered.
    lay = app.CreateLayout("linear", "VCenter,FillXY");

    btn = app.CreateButton("Connect", 0.4, 0.15);
    btn.SetOnTouch(btn_OnTouch);
    lay.AddChild(btn);

    text = app.CreateText("Speed")
    lay.AddChild(text);

    skb = app.CreateSeekBar(0.8);
    skb.SetRange(1000);
    skb.SetValue(500);
    skb.SetOnTouch(skb_OnTouch);
    lay.AddChild(skb);

    text1 = app.CreateText("Angle")
    lay.AddChild(text1);

    skb1 = app.CreateSeekBar(0.8);
    skb1.SetRange(1800);
    skb1.SetValue(900);
    skb1.SetOnTouch(skb1_OnTouch);
    lay.AddChild(skb1);

    //Add layout to app.    
    app.AddLayout(lay);
    app.AddLayout(lay_diag);

    //Create Bluetooth serial object. 
    bt = app.CreateBluetoothSerial();
    bt.SetOnConnect(bt_OnConnect);
    bt.SetOnReceive(bt_OnReceive);
    bt.SetSplitMode("End", "\n");

    //Create the stop button
    stop_btn = app.CreateButton("STOP", 0.4, 0.15, "lego");
    stop_btn.SetOnTouch(stop_btn_OnTouch);
    lay.AddChild(stop_btn);
}

//Called when user touches the connect button. 
function btn_OnTouch() {
    bt.Connect("SerialBT");
}

//Called when we are connected. 
function bt_OnConnect(ok) {
    if (ok) {
        setTimeout(communicate, 100);
        //hide the connect button
    }
    else app.ShowPopup("Failed to connect!");
}

//Called when receiving data via bluetooth
function bt_OnReceive(data) {
    //app.ShowPopup("data="+data);
    var data_array = data.split(",");
    diag_text_voltage.SetText("Voltage: " + data_array[0]);
    diag_text_current.SetText("Current: " + data_array[1]);
}

//Speed control
function skb_OnTouch(speed) {
    //app.ShowPopup("speed="+speed);
    _speed = speed;
}

// Steering control
function skb1_OnTouch(angle) {
    //app.ShowPopup("angle="+angle);
    _angle = angle
}

//timed comunication function
function communicate() {
    bt.Write(_speed + "\n");
    bt.Write(_angle + "\n");
    setTimeout(communicate, 100);
}

//called when the user touches the stop button
function stop_btn_OnTouch() {
    skb.SetValue(500);
    skb1.SetValue(900);
    _speed = 500;
    _angle = 900;
}


