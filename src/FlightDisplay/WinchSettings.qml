import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window
import QtQml.Models

import QGroundControl

import QGroundControl.Controls
import com.Winch 1.0

Rectangle
{
    id:winchdisplay
    property real   _toolsMargin:           ScreenTools.defaultFontPixelWidth * 0.75
    property real motor_value: 0.0
    property real actuator_value: 0.0
    width:parent.width
    height:parent.height
    signal closed;

    color: "transparent"
    //opacity: 0.4

    Rectangle
    {
        width:parent.width
        height:parent.height
        signal closed;

        color: "black"
        opacity: 0.4
        radius:0.01*parent.width

    }
    QGCButton {
        id: close
        anchors
        {
            right:parent.right
            top:parent.top
            margins:_toolsMargin
        }

        contentItem: Text {
            id: response_button2
            text: "Exit"
            font.pixelSize: Math.min(winchdisplay.width / 40,
                                     winchdisplay.height / 30)
            font.bold: true
            //font.pixelSize: font_size
            style: Text.Sunken
            color: "White"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
        background: Rectangle
        {
            color:"#4287f5"
            radius:0.005*parent.width
            border.color:"#000000"
            border.width:0.05*parent.width
        }

        MouseArea
        {
            anchors.fill: parent

            onClicked:
            {
                closed()
            }
        }


    }

    ColumnLayout
    {
        id:column

        width:0.9*parent.width
        height:0.75*parent.height
        spacing: 0.025 * parent.height
        clip: true
        anchors
        {
            left:parent.left
            top:close.bottom
            margins:_toolsMargin
        }

        RowLayout
        {
            id:row1
            Layout.fillHeight: true
            spacing: 0.005 * parent.width

            Text {
                //Layout.fillWidth: true
                text: "Motor                          "
                font.family: "Segoe UI Emoji"
                font.bold: true
                //horizontalAlignment: Text.AlignHCenter
                //style: Text.Outline
                color: "white"
                font.pixelSize: Math.min(winchdisplay.width / 30,
                                         winchdisplay.height / 20)
            }



            QGCButton
            {
                id:forward1
                contentItem: Text {
                    id: response_button5
                    text: "Forward"
                    font.pixelSize: Math.min(winchdisplay.width / 30,
                                             winchdisplay.height / 20)
                    font.bold: true
                    //font.pixelSize: font_size
                    style: Text.Sunken
                    color: "White"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                background: Rectangle
                {
                    color:"#4287f5"
                    radius:0.005*parent.width
                    border.color:"#000000"
                    border.width:0.05*parent.width
                }
                MouseArea
                {
                    anchors.fill: parent

                    onClicked:
                    {
                        //_winch.run_motor(1)
                        _winch.winch_motor_actuator(0,1);
                        motor_value=1;

                    }
                }
            }

            QGCButton
            {
                id:stop1
                contentItem: Text {
                    id: response_button4
                    text: "Stop"
                    font.pixelSize: Math.min(winchdisplay.width / 30,
                                             winchdisplay.height / 20)
                    font.bold: true
                    //font.pixelSize: font_size
                    style: Text.Sunken
                    color: "White"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                background: Rectangle
                {
                    color:"#4287f5"
                    radius:0.005*parent.width
                    border.color:"#000000"
                    border.width:0.05*parent.width
                }
                MouseArea
                {
                    anchors.fill: parent

                    onClicked:
                    {
                        //_winch.run_motor(0)
                        _winch.winch_motor_actuator(0,0);
                        motor_value = 0;

                    }
                }
            }

            QGCButton
            {
                id:backward1
                contentItem: Text {
                    id: response_button3
                    text: "Backward"
                    font.pixelSize: Math.min(winchdisplay.width / 30,
                                             winchdisplay.height / 20)
                    font.bold: true
                    //font.pixelSize: font_size
                    style: Text.Sunken
                    color: "White"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                background: Rectangle
                {
                    color:"#4287f5"
                    radius:0.005*parent.width
                    border.color:"#000000"
                    border.width:0.05*parent.width
                }
                MouseArea
                {
                    anchors.fill: parent

                    onClicked:
                    {
                        //_winch.run_motor(2)
                        _winch.winch_motor_actuator(0,2);
                        motor_value = 2;

                    }
                }
            }

        }

        RowLayout
        {
            id:row2
            Layout.fillHeight: true

            spacing:row1.spacing// 0.25 * parent.width

            Text {
                //Layout.fillWidth: true
                text: "Actuator                      "
                font.family: "Segoe UI Emoji"
                font.bold: true
                color: "white"
                /*font.pixelSize: Math.min(winchdisplay.width / 60,
                                         winchdisplay.height / 50)*/
                font.pixelSize: Math.min(winchdisplay.width / 30,
                                         winchdisplay.height / 20)
            }



            QGCButton
            {
                id:forward2
                contentItem: Text {
                    id: response_button7
                    text: "Forward"
                    font.pixelSize: Math.min(winchdisplay.width / 30,
                                             winchdisplay.height / 20)
                    font.bold: true
                    //font.pixelSize: font_size
                    style: Text.Sunken
                    color: "White"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                background: Rectangle
                {
                    color:"#4287f5"
                    radius:0.005*parent.width
                    border.color:"#000000"
                    border.width:0.05*parent.width
                }
                MouseArea
                {
                    anchors.fill: parent

                    onClicked:
                    {
                        //_winch.run_actuator(1)
                        _winch.winch_motor_actuator(1,1);
                        actuator_value = 1;
                    }
                }
            }
            QGCButton
            {
                id:stop2
                contentItem: Text {
                    id: response_button6
                    text: "Stop"
                    font.pixelSize: Math.min(winchdisplay.width / 30,
                                             winchdisplay.height / 20)
                    font.bold: true
                    //font.pixelSize: font_size
                    style: Text.Sunken
                    color: "White"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                background: Rectangle
                {
                    color:"#4287f5"
                    radius:0.005*parent.width
                    border.color:"#000000"
                    border.width:0.05*parent.width
                }
                MouseArea
                {
                    anchors.fill: parent

                    onClicked:
                    {
                        //_winch.run_actuator(0)
                        _winch.winch_motor_actuator(1,0);
                        actuator_value = 0;

                    }
                }
            }
            QGCButton
            {
                id:backward2
                contentItem: Text {
                    id: response_button8
                    text: "Backward"
                    font.pixelSize: Math.min(winchdisplay.width / 30,
                                             winchdisplay.height / 20)
                    font.bold: true
                    //font.pixelSize: font_size
                    style: Text.Sunken
                    color: "White"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                background: Rectangle
                {
                    color:"#4287f5"
                    radius:0.005*parent.width
                    border.color:"#000000"
                    border.width:0.05*parent.width
                }
                MouseArea
                {
                    anchors.fill: parent

                    onClicked:
                    {
                        //_winch.run_actuator(2)
                        _winch.winch_motor_actuator(1,2);
                        actuator_value=2;
                    }
                }
            }

        }

        RowLayout
        {
            id:row3
            Layout.fillHeight: true

            spacing:row1.spacing //0.25 * parent.width

            ColumnLayout
            {
                id:column2
                Layout.fillWidth: true
                spacing: 0.15 * parent.height

                Text {
                    Layout.fillHeight: true
                    text: "Winch Encoder Value"
                    font.family: "Segoe UI Emoji"
                    font.bold: true
                    color: "white"
                    font.pixelSize: Math.min(winchdisplay.width / 30,
                                             winchdisplay.height / 20)
                }
                Text {
                    Layout.fillHeight: true
                    id:a1
                    text: motor_value
                    font.family: "Segoe UI Emoji"
                    font.bold: true
                    horizontalAlignment: Text.AlignHCenter
                    style: Text.Outline
                    color: "white"
                    font.pixelSize: Math.min(winchdisplay.width / 10,
                                             winchdisplay.height / 5)
                }
            }

            /*ColumnLayout
            {
                id:column3

                width:parent.width
                height:parent.width
                spacing: 0.075 * parent.height

                Item {
                    Layout.fillWidth: true
                    width: 0.05 * parent.width
                    Text {
                        text: "Actuator"
                        font.family: "Segoe UI Emoji"
                        font.bold: true
                        horizontalAlignment: Text.AlignHCenter
                        style: Text.Outline
                        color: "white"
                        font.pixelSize: Math.min(parent.width / 90,
                                                 parent.height / 80)
                    }
                }

                Item {
                    Layout.fillWidth: true
                    width: 0.05 * parent.width
                    Text {
                        text: "Actuator"
                        font.family: "Segoe UI Emoji"
                        font.bold: true
                        horizontalAlignment: Text.AlignHCenter
                        style: Text.Outline
                        color: "white"
                        font.pixelSize: Math.min(parent.width / 90,
                                                 parent.height / 80)
                    }
                }
            }*/


        }
    }

    Winch
    {
        id:_winch
    }

    Connections
    {
        target: _winch

        function onData_to_be_updated(value)
        {
            var data = value;
            //console.log(data)
            motor_value=data
        }
    }

}
