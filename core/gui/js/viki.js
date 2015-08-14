var modules;
var jsPlumbInstance;  // to make instance globally available
var modulesInCanvas = [];

$(document).ready(function(){
    // All links with an id starting with viki are buttons that expect a reaction from python. This process is automated: the python function with name equal to the id will run.
    $("a[id^=viki]").click(function(){
        statusmessage = $(this).data("statusmessage");
        if(typeof(statusmessage) != "undefined")
        {
            updateStatus(statusmessage);
        }
        send(JSON.stringify({name: $(this).attr('id'), value: false}));
        return false;
    });

    $("#saveConfigXML").click(function(){
        writeConfig(getConfig());
    });

    $("#saveConfigLaunch").click(function(){
           send(JSON.stringify({name: "vikiConfigLaunch", value: false}));
       });

    // Manually request first module list.
    updateStatus('Asking for initial module list');
    send(JSON.stringify({name: "vikiRefreshModules", value: false}));
});

function updateStatus(msg) {
    var dt = new Date();
    var time =('0'  + dt.getHours()).slice(-2)+':'+('0' + dt.getMinutes()).slice(-2)+':'+('0' + dt.getSeconds()).slice(-2);
    $('#statusLabel').prepend(time + " - " + msg + "<br />");
}

function updateModules(modulelist) {
    updateStatus('Received modules');
    modules = modulelist;
    showModulesInPalette(modules);
    initPalette();
    updateStatus('Updated module panel')
}

function showModulesInPalette(modules) {
    $('#palette #list').html("");
    modules.forEach(function(module){
        $('#palette #list').append('<li class="module_palette" id="'+module.id+'"><img src="img/plugin.png" />'+module.id+'</li>');
    });
} 

function initPalette() {
    $(".module_palette").attr({
        "draggable" : "true"
    });

    $(".module_palette").on("dragstart", startDrag);

    $(".project-container").attr({
        "ondragover" : "allowDrop(event)",
        "ondrop" : "dropModule(event)"
    });
    
}

function enableStartCore() {
    $("#vikiStopRosCore").parent().hide();
    $("#vikiStartRosCore").parent().show();
}

function enableStopCore() {
    $("#vikiStopRosCore").parent().show();
    $("#vikiStartRosCore").parent().hide();
}

// this is the paint style for the connecting lines..
var connectorPaintStyle = {
        lineWidth: 4,
        strokeStyle: "#61B7CF",
        joinstyle: "round",
        outlineColor: "white",
        outlineWidth: 2
    },
// .. and this is the hover style.
    connectorHoverStyle = {
        lineWidth: 4,
        strokeStyle: "#216477",
        outlineWidth: 2,
        outlineColor: "white"
    },
    endpointHoverStyle = {
        fillStyle: "#216477",
        strokeStyle: "#216477"
    };

jsPlumb.ready(function () {

    jsPlumbInstance = jsPlumb.getInstance({
        // default drag options
        DragOptions: { cursor: 'pointer', zIndex: 2000 },
        // the overlays to decorate each connection with.  note that the label overlay uses a function to generate the label text; in this
        // case it returns the 'labelText' member that we set on each connection in the 'init' method below.
        ConnectionOverlays: [
            [ "Arrow", { location: 1 } ],
            [ "Label", {
                location: 0.1,
                id: "label",
                cssClass: "aLabel"
            }]
        ],
        Container: "project-container"
    });

    var basicType = {
        connector: "StateMachine",
        paintStyle: { strokeStyle: "red", lineWidth: 4 },
        hoverPaintStyle: { strokeStyle: "blue" },
        overlays: [
            "Arrow"
        ]
    };
    jsPlumbInstance.registerConnectionType("basic", basicType);

    // suspend drawing and initialise.
    jsPlumbInstance.batch(function () {

        // listen for new connections; initialise them the same way we initialise the connections at startup.
        jsPlumbInstance.bind("beforeDrop", function (connInfo) {
            sourceType = connInfo.connection.endpoints[0].getParameter("type");
            targetType = connInfo.dropEndpoint.getParameter("type");

            console.log(sourceType, targetType);
            if (sourceType != targetType) {
                console.log("Connection endpoints are not of the same type. Removing..");
                return false;
            }

            return true;
        });

        // make all the window divs draggable
        jsPlumbInstance.draggable($(".project-container .window"), { grid: [20, 20] });

        //
        // listen for clicks on connections, and offer to delete connections on click.
        //
        jsPlumbInstance.bind("click", function (conn, originalEvent) {
            // if (confirm("Delete connection from " + conn.sourceId + " to " + conn.targetId + "?"))
            //   instance.detach(conn);
            //conn.toggleType("basic");
        });
    });

    jsPlumb.fire("jsPlumbDemoLoaded", jsPlumbInstance);

});


function addInputsToWindow(moduleId, inputs) {
    var targetEndpoint = {
        endpoint: "Dot",
        paintStyle: { fillStyle: "#7AB02C", radius: 11 },
        hoverPaintStyle: endpointHoverStyle,
        maxConnections: -1,
        dropOptions: { hoverClass: "hover", activeClass: "active" },
        isTarget: true
    };

    for (var i = 0; i < inputs.length; i++) {
        var anchorId = moduleId + "input" + i.toString;
        var pos = [0, ((i+1)/(inputs.length +1)), -1, 0];
        jsPlumbInstance.addEndpoint(moduleId, targetEndpoint, {
            anchor: pos,
            parameters: {
                type: inputs[i].message_type
            },
            overlays: [
                [ "Label", {
                    location: [-1.5, 0.5],
                    label: inputs[i].name,
                    cssClass: "endpoint-label"
                } ]
            ],
            parameters: {
                name: inputs[i].name
            }
        });
    }
}

function addOutputsToWindow(moduleId, outputs) {

    var sourceEndpoint = {
        endpoint: "Dot",
        paintStyle: {
            strokeStyle: "#7AB02C",
            fillStyle: "transparent",
            radius: 7,
            lineWidth: 3
        },
        isSource: true,
        connector: [ "Flowchart", { stub: [40, 60], gap: 10, cornerRadius: 5, alwaysRespectStubs: true } ],
        connectorStyle: connectorPaintStyle,
        hoverPaintStyle: endpointHoverStyle,
        connectorHoverStyle: connectorHoverStyle,
        dragOptions: {}
    };

    for (var i = 0; i < outputs.length; i++) {
        var anchorId = moduleId + "output" + i.toString;
        var pos = [1, ((i+1)/(outputs.length +1)), 1, 0];
        jsPlumbInstance.addEndpoint(moduleId, sourceEndpoint, {
            anchor: pos,
            parameters: {
                type: outputs[i].message_type
            },
            overlays: [
                [ "Label", {
                    location: [-1.5, 0.5],
                    label: outputs[i].name,
                    cssClass: "endpoint-label"
                } ]
            ],
            parameters: {
                name: outputs[i].name
            }
        });
    }
};

function startDrag(ev) {
    ev = ev.originalEvent;
    updateStatus("dragging object: " + ev.target.id);
    ev.dataTransfer.setData("moduleId", ev.target.id);
}

function allowDrop(ev) {
    // updateStatus("allowing drop now..");
    ev.preventDefault();
}

function dropModule(ev) {
    updateStatus("dropped a module to the project-container");
    ev.preventDefault();

    // module id:
    var data = ev.dataTransfer.getData("moduleId");
    var modId = data;
    var uModId = guid();  // generate unique id
    
    $(".project-container").append('<div class="window" id="'+uModId+'"><span class="window_label">'+modId+'</span></div>');

    // add to inCanvasArray
    var modToAdd = getModuleById(modId);
    modToAdd.uWindowId = uModId;
    modulesInCanvas.push(modToAdd);

    // start module at correct position
    var width = 100;
    var height = 20 + Math.max(modToAdd.inputs.length, modToAdd.outputs.length) * 20;
    var X = ev.pageX - 0.5*width;
    var Y = ev.pageY - 0.5*height;

    $(".project-container #"+uModId).offset({
        top : Y,
        left: X
    }).width(width)
        .height(height);

    // make draggable
    jsPlumbInstance.draggable($(".project-container .window"), { grid: [20, 20] });

    // connections
    addInputsToWindow(uModId, modToAdd.inputs);
    addOutputsToWindow(uModId, modToAdd.outputs);

}

function getModuleById(Id) {
    for (var i=0; i <modules.length; i++) {
        if (modules[i].id == Id) {
            return modules[i];
        }
    }
}

function getModuleByUWindowId(uId) {
    for (var i=0; i<modulesInCanvas.length; i++) {
        if (modulesInCanvas[i].uWindowId == uId) {
            return modulesInCanvas[i];
        }
    }
}

function guid() {
    // generates a unique number
    return ("0000" + (Math.random()*Math.pow(36,4) << 0).toString(36)).slice(-4);
}

function deleteWindowFromCanvas(uId) {
    // remove from jsPlumb
    jsPlumbInstance.remove(uId);
    // remove from array
    for (var i=0; i<modulesInCanvas.length; i++) {
        if (modulesInCanvas[i].uWindowId == uId) {
            modulesInCanvas.splice(i, 1);
        }
    }
    // update status
    updateStatus("window removed!");
}

function getConfig() {
    var config = {};
    config.modsToAdd = [];
    config.connectsToAdd = [];
    $(".project-container").children().each(function() {
        uId = this.id;  // unique id for this element
        var mod = {};  // initialize module object
        
        // save all modules (only use role, type and id)
        var tempmod = getModuleByUWindowId(uId);  // get infor from unique id        
        mod.id = uId;  // save id
        mod.type = tempmod.id;  // save type
        mod.role = tempmod.type;  // save unique id
        config.modsToAdd.push(mod);  // add to list of modules to add

        // all connections for this module
        var connects = jsPlumbInstance.getAllConnections(uId);
        for (var i=0; i<connects.length; i++) {
            var connectionToAdd = {};  // init connect to add object
            
            var connect = connects[i];  // this connect
            if (connect.targetId == uId) {  // only write connection if this module is target (avoids doubles)
                for (var j=0; j<connect.endpoints.length; j++) {  // loop across the (two) enpoints
                    endpoint = connect.endpoints[j];
                    // get source or target, prepend with module id and "/"
                    if (endpoint.isSource) {
                        connectionToAdd.pub = endpoint.elementId + "/" + endpoint.getParameter("name");
                    } else if (endpoint.isTarget) {
                        connectionToAdd.sub = endpoint.elementId + "/" + endpoint.getParameter("name");
                    } else {
                        console.log('Found an enpoint that is not a source, nor a target');
                        alert('error, please check console.log');
                    }                
                }    

                // save to connectionlist
                config.connectsToAdd.push(connectionToAdd);
            }
            
        }
    });
    return config;
}

function writeConfig(config) {
    // create config XML 
    var configXML = document.createElement("configuration");
    configXML.setAttribute("id", "VIKI-imported-config");

    // add modules to the config XML
    for (var i=0; i<config.modsToAdd.length; i++) {
        var modXML = document.createElement(config.modsToAdd[i].role);
        modXML.setAttribute("type", config.modsToAdd[i].type);
        modXML.setAttribute("id", config.modsToAdd[i].id);
        configXML.appendChild(modXML);
    }

    // add connects to the config XML
    for (var i=0; i<config.connectsToAdd.length; i++){
        var connectXML = document.createElement("connect");
        connectXML.setAttribute("publisher", config.connectsToAdd[i].pub);
        connectXML.setAttribute("listener", config.connectsToAdd[i].sub);
        configXML.appendChild(connectXML);
    }
    
    send(JSON.stringify({name: "vikiConfigXML", value: configXML.outerHTML}));
}