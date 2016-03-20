var modules;  // List of available modules
var jsPlumbInstance;  // Instance of the JsPlumb state
var modulesInCanvas = [];  // list of the modules currently on canvas
var selectedModuleUid = null;  // Unique id of currently selected module
var generatedGUIDs = [];  // List of used id's. A new one is checked against this
var history = [];  // List containing max. 10 previous states to ctrl+z to
var future = [];  // List containing max. 10 future states to ctrl+y to
var localHostName = "localhost";  // Name of this computer
var machines = {};  // List of machines

$(document).ready(function(){
    // prevent dragging event of viki logo
    $('img').on('dragstart', function (event) {event.preventDefault()});
    

    // All links with an id starting with viki are buttons that expect a reaction from python.
    // This process is automated: the python function with name equal to the id will run.
    $("a[id^=viki]").click(function(){
        statusmessage = $(this).data("statusmessage");
        if(typeof(statusmessage) != "undefined")
        {
            updateStatus(statusmessage);
        }

        datafunction = $(this).data("function");
        if(typeof(datafunction) != "undefined")
        {
            value = window[datafunction]();
        }
        else
        {
            value = false;
        }

        send(JSON.stringify({name: $(this).attr('id'), value: value}));
        return false;
    });

    // connect buttons to callbacks oustide the aforementioned automated method
    $("#reset").click(canvasReset);
    $("#stepBack").click(stepBack);
    $("#stepForward").click(stepForward);
    $("#makeNoRun").click(function(){
        updateStatus("Reqesting make...");
        send(JSON.stringify({name: "vikiMakeNoRun", value: getConfigXML(getConfig())}));
    });
    $("#makeAndRun").click(function(){
        updateStatus("Requesting make and run...");
        send(JSON.stringify({name: "vikiMakeAndRun", value: getConfigXML(getConfig())}));
       });
    $('#module-filter-text').on('keyup change', filterModules);

    $("#italian").click(function(){
        // Italian language support:
        // runs a supermario image across the screen as a joke for Matteo
        
        // get size of container
        var w = $('.project-container').width();
        var h = $('.project-container').height();
        // put image in page
        $('.project-container').prepend('<img id="marioImg" src="img/mario.png" />');
        // set start position
        $("#marioImg").css({
            position: "absolute",
            top: (0.5*h) + "px",
            left: 0 + "px"
            }).show();
        // move to right (calls delete function afterwards)
        $("#marioImg").animate({left: w + 'px'}, 3000, "linear", removeMario);
        updateStatus("Got you! Of course there is no Italian language support.");
    });
    /**
     * Removes the mario image form the screen
     * has to be in a separate function to allow the screen to redraw
     * before removing the mario
     */
    function removeMario() {
        // function to remove the mario added when italian language support is used
        $("#marioImg").remove();
    }

    $("#logo").on('click', function() {
    // Makes the VIKI-logo re-fly in when clicking on it 
         var el     = $(this),  
             newone = el.clone(true);
                   
         el.before(newone);
         el.remove();
    });

    $("#editLocalHostName").click(function() {
    // Change the hostname of this machine via a dialog
        var hostName = window.prompt("Please enter the new local hostname.",localHostName);
        if(hostName  != null) {
            setLocalHostName(hostName);
        }
        return false;
    });

    $("#addMachine").click(function() {
    // add a machine to to list of machines
        $("#machineForm input[name='prevname']").val('');
        $("#machineForm input[name='name']").val('');
        $("#machineForm input[name='hostname']").val('');
        $("#machineForm input[name='username']").val('');
        $("#machineForm input[name='password']").val('');
    });

    $("#saveMachine").click(function(){
    // save changes made to a machine
        oldName = $("#machineForm input[name='prevname']").val();
        name = $("#machineForm input[name='name']").val();
        hostname = $("#machineForm input[name='hostname']").val();
        username = $("#machineForm input[name='username']").val();
        password = $("#machineForm input[name='password']").val();

        if(oldName != name && oldName != '')
        {
            delete machines[oldName];
        }
        // Update existing
        machines[name] = {
            name: name,
            hostname: hostname,
            username: username,
            password: password
        };

        syncMachineList(machines);
    });

    $("#machineList").on("click",".editMachine",function(event){
    // shows the popup window to change a machine name
        key = event.currentTarget.dataset.key;
        $("#machineForm input[name='prevname']").val(machines[key].name);
        $("#machineForm input[name='name']").val(machines[key].name);
        $("#machineForm input[name='hostname']").val(machines[key].hostname);
        $("#machineForm input[name='username']").val(machines[key].username);
        $("#machineForm input[name='password']").val(machines[key].password);
        $('#machineAddPopup').modal('show');
        return false;
    });

    $("#machineList").on("click",".deleteMachine",function(event){
    // Delete a machine
        delete machines[event.currentTarget.dataset.key];
        syncMachineList(machines);
        return false;
    });

    // Set machine list
    syncMachineList(machines);

    // Manually request first module list.
    updateStatus('Asking for initial module list');
    send(JSON.stringify({name: "vikiRefreshModules", value: false}));
});

/**
 * Save the current state of the project
 */
function saveState() {
    history.push(getProject());
    if(history.length > 10) {
        history.shift();
    }
}

/**
 * Go back one step in the history (this is ctrl+z)
 */
function stepBack() {
    if(history.length > 0)
    {
        savedState = history.pop();
        future.splice(0, 0, savedState);
        openFromJSON(JSON.parse(savedState));
    }
}

/**
 * Go forward one step into the future (this is ctrl+y)
 */
function stepForward() {
    if(future.length > 0)
    {
        savedState = future.shift();
        history.push(savedState);
        openFromJSON(JSON.parse(savedState));
    }
}

/**
 * Get all information necessary to restore the state of the screen.
 * this information is sufficient for stepping back in history with ctrl+z
 * or to save the configuration
 * @returns JSON stringified state of the window
 */
function getProject() {
    var nodes = [];
    $(".window").each(function (idx, elem) {
        var $elem = $(elem);
        var endpoints = jsPlumbInstance.getEndpoints($elem.attr('id'));
        nodes.push({
            blockId: $elem.attr('id'),
            nodetype: $elem.attr('data-nodetype'),
            positionX: parseInt($elem.offset().left, 10),
            positionY: parseInt($elem.offset().top, 10)
        });
    });
    var connections = [];
    $.each(jsPlumbInstance.getConnections(), function (idx, connection) {
        connections.push({
            sourceUuid: connection.endpoints[0].getUuid(),
            targetUuid: connection.endpoints[1].getUuid()
        });
    });

    var jsp = {};
    jsp.nodes = nodes;
    jsp.connections = connections;

    project = {
        modulesInCanvas: modulesInCanvas,
        machines: machines,
        localHostName: localHostName,
        generatedGUIDs: generatedGUIDs,
        modules: jsp
    };
    return JSON.stringify(project);
}

/**
 * Moves an element to a new position on the screen and redraws jsPlumb
 * @param id id of the element to move
 * @param posX the new horizontal position of the element
 * @param posY the new vertical position of the element
 */
function repositionElement(id, posX, posY){
    $('#'+id).css('left', posX);
    $('#'+id).css('top', posY);
    jsPlumb.repaint(id);
}

/**
 * Clears the canvas and the modulelist
 * Machines are unchanged
 */
function canvasReset() {
    $('.window').remove();
    jsPlumbInstance.reset();
    modulesInCanvas = [];
    generatedGUIDs = [];
}

/**
 * Opens a JSON string containing a window state
 * the JSON string can be generated by getProject()
 * @param project The JSON string
 */
function openFromJSON(project) {
    canvasReset();
    var flowChart = project.modules;
    var nodes = flowChart.nodes;
    $.each(nodes, function( index, elem ) {
        addModuleToContainer(elem.blockId, elem.positionX, elem.positionY, elem.blockId);
    });

    var connections = flowChart.connections;
    $.each(connections, function( index, elem ) {
        jsPlumbInstance.connect({
            uuids: [elem.sourceUuid, elem.targetUuid]
        });
    });

    modulesInCanvas = project.modulesInCanvas;
    generatedGUIDs = project.generatedGUIDs;
    machines = project.machines;
    syncMachineList(machines);
    localHostName = project.localHostName;
    setLocalHostName(localHostName);
    jsPlumbInstance.repaintEverything();
    updateStatus('End of open / restore.')
}

/**
 * Print a status message to screen
 * including a timestring
 * @param msg The message to print
 */
function updateStatus(msg) {
    var dt = new Date();
    var time =('0'  + dt.getHours()).slice(-2)+':'+('0' + dt.getMinutes()).slice(-2)+':'+('0' + dt.getSeconds()).slice(-2);
    $('#statusLabel').prepend(time + " - " + msg + "<br />");
}

/**
 * Updates the palette with a new module list
 * @param modulelist a list of modules to replace the ones in the palette with
 */
function updateModules(modulelist) {
    updateStatus('Received modules.');
    modules = modulelist;
    modules.sort(function(x, y) {
        if (x.type < y.type) return -1;
        if (y.type < x.type) return 1;
        return 0;
    });
    showModulesInPalette(modules);
    initPalette();
    updateStatus('Updated module panel.')
}

/**
 * Places modules in the palette and displays them
 * @param modules modules to show
 */
function showModulesInPalette(modules) {
    $('#palette #list').html("");
    modules.forEach(function(module){
        // Get a fancy image for the module, with a lot of fallbacks
        var icon_path = 'img/plugin.png';
        var image_html;
        if (module.meta.icon != null) {
            if (module.meta.icon.indexOf("glyphicon") == 0) {
                image_html = "<i class='glyphicon "+module.meta.icon+"'></i>"
            } else {
                var icon_path = '../../' + module.path.substring(0, module.path.lastIndexOf("/")) + '/' + module.meta.icon;    
            }
        }
        if (image_html == null) {
            image_html = '<img src="'+icon_path+'" />';
        }

        $('#palette #list').append('<li class="module_palette '+module.type+'" id="'+module.id+'">'+
            image_html+' <h3>'+module.meta.name+'</h3>'+
            '<p class="description">'+module.meta.description+'</p>'+
            '<p class="type">type: '+module.type+'</p>'+
            '</li>');        
    });

    var module_els = $(".module_palette");
    module_els.attr({"draggable" : "true"});
    module_els.on("dragstart", startDrag);
}

/**
 * Filter modules based in keyboard input
 * @param event keyboard event
 */
function filterModules(event) {
    showModulesInPalette([]);

    var filter = $(this).val().toLowerCase();
    if (filter == '') {
        $('.module_palette').slideDown(modules);
    }

    filtered_modules = [[], [], [], []];

    // check for each module if it maches the searching criteria,
    // and sort these in the right array, based on priority of the search field
    modules.forEach(function(module) {
        search_fields = [module.meta.name, module.id, module.meta.description, module.type];
        for (var i=0; i<search_fields.length; i++) {
            sfield = search_fields[i];
            if (sfield == undefined) continue;
            if (sfield.toLowerCase().indexOf(filter) != -1) {
                filtered_modules[i].push(module);
                break;
            }
        }
    });

    modules_to_show = filtered_modules[0].concat(filtered_modules[1], filtered_modules[2], filtered_modules[3]);
    showModulesInPalette(modules_to_show);

    if (event.which == 13) {

        if (modules.length == 1) {
            var x = 50 + $('.project-container').offset().left + Math.random() * ($('.project-container').width() - 100);
            var y = 50 + $('.project-container').offset().top + Math.random() * ($('.project-container').height() - 100);
            addModuleToContainer(modules_to_show[0].id, x, y);
        }
    } 
}

/**
 * Sets up the palette (which holds the modules)
 * to allow for dragging of modules, configure the dragstart event
 * and makes the canvas accept the modules as a drop
 */
function initPalette() {
    $(".module_palette").attr({
        "draggable" : "true"
    });

    $(".module_palette").on("dragstart", startDrag);

    $(".project-container").attr({
        "ondragover" : "allowDrop(event)",
        "ondrop" : "dropModule(event)"
    });
    
    $(document).on('click', '.window', onWindowClick);
    $(document).on('keydown', keyPressed);
    $(document).on('module_selected', onModuleSelect);
}

/**
 * enables the button to start the roscore
 * disables the button to stop the roscore
 */
function enableStartCore() {
    $("#vikiStopRosCore").hide();
    $("#vikiStartRosCore").show();
}

/**
 * disables the button to start the roscore
 * enables the button to stop the roscore
 */
function enableStopCore() {
    $("#vikiStopRosCore").show();
    $("#vikiStartRosCore").hide();
}

/**
 * Select a module
 * also clears the currently selected module if a new module is selected
 */
function onWindowClick(event) {
    var willSelect = ( selectedModuleUid !== $(this).attr('id') ); // only select if we click a non-selected module
    clearSelectedModule(); // make sure we only select a single module
    if (willSelect) {
        selectedModuleUid = $(this).attr('id');    
        $(this).attr('selected', 'selected');
        $(document).trigger("module_selected");
    }
}

/**
 * Clears the selected module, so that nothing is selected anymore
 */
function clearSelectedModule() {
    selectedModuleUid = null;
    $('.window').removeAttr('selected');
    // clear the properties pane
    $('#selectedWindowProperties tbody').empty();
    $('p#selectedWindowInfo').empty();
    // set text instead
    $('#selectedWindowProperties tbody').append('<tr><td colspan="2">No module selected.</td></tr>');
}

/** 
 * Delete the currently selected module from the canvas
 */
function deleteSelectedModule() {
    if (selectedModuleUid != null) {
        saveState();
        deleteWindowFromCanvas(selectedModuleUid);
        clearSelectedModule();
        updateStatus("Deleted module from canvas.");
    }
}

/**
 * Callback for keypressed events
 * implements several keyboard shortcuts
 */
function keyPressed(event) {
    switch (event.which) {
        case 46:  // delete
        case 8:  //backspace
            if(document.activeElement.type != 'text')
            {
                deleteSelectedModule();
            }
            break;
        case 77:  // m
            if (event.ctrlKey) {
                $('#module-filter-text').focus();
            }
            break;
        case 90:  // z
            if (event.ctrlKey) {
                stepBack();
            }
            break;
        case 89:  // y
            if (event.ctrlKey) {
                stepForward();
            }
    }
}

/**
 * Populates the right pane with module properties and settings
 * such as the parameters, arguments, and machine selection buttons
 */
function onModuleSelect(event) {
    var selectedModule = getModuleByUWindowId(selectedModuleUid);
    $('p#selectedWindowInfo').html("<h3>"+selectedModule.meta.name+"</h3>"+
            "<button class='btn btn-default' id='argButton' data-toggle='modal' data-target='#argPopup'>Add/edit arguments</button>"+
            "<br><button class='btn btn-default' id='prefixButton' data-toggle='modal' data-target='#prefixPopup'>Add/edit prefixes</button>"+
            "<br><button class='btn btn-default' id='machineSelectButton' data-toggle='modal' data-target='#machineSelectPopup'>Select machine</button></br>"+
            "<br><strong>Uid: </strong>"+selectedModule.uWindowId+"<br/>");
    var tbody = $('#selectedWindowProperties tbody');
    tbody.empty();

    // for command line arguments (open the modal, edit it's content)
    $('#argButton').click(function() {
        saveState();
        // clear list of executables
        $("#argPopupBody > table > tbody > tr").not(":first").remove();

        // make list of executables
        for(var i=0; i<selectedModule.executables.length; i++){
            var texec = selectedModule.executables[i];
            var originalCmd;
            if (typeof selectedModule.args[i] === "undefined") {
                originalCmd = "";
            } 
            else {
                originalCmd = selectedModule.args[i].cmd;
            }
            var tc = "<tr><td>"+texec.id+"</td><td><input type='text' class='form-control' value=" + originalCmd + "></input></td></tr>";
            $("#argPopupBody > table > tbody").append(tc);
        }

        // Remove old click bind first
        $("#saveArgButton").unbind('click');
        $("#saveArgButton").click(function() {
            // hide dialog
            $("#argPopup").modal("hide");

            // save arguments
            selectedModule.args = [];  // empty list
            for (var i=0; i< $('#argPopupBody > table > tbody > tr').not(":first").length; i++) {
                var arg = {};
                arg.execId = $($('#argPopupBody > table > tbody > tr')[i+1].children[0]).text();
                arg.cmd =    $($('#argPopupBody > table > tbody > tr')[i+1].children[1].children[0]).val();
                selectedModule.args.push(arg);
            }

            updateStatus("Saved arguments");
        });
    });

    // for launch-prefixes
    $('#prefixButton').click(function() {
        saveState();
        // clear list of executables
        $("#prefixPopupBody > table > tbody >tr").not(":first").remove();

        // make list of executables
        for(var i=0; i<selectedModule.executables.length; i++){
            var texec = selectedModule.executables[i];
            var originalPrefix;
            if (typeof selectedModule.prefixes[i] === "undefined") {
                originalPrefix = "";
            } 
            else {
                originalPrefix = selectedModule.prefixes[i].prefix;
            }
            var tc = "<tr><td>"+texec.id+"</td><td><input type='text' class='form-control' value=" + originalPrefix + "></input></td></tr>";
            $("#prefixPopupBody > table > tbody").append(tc);
        }

        // Remove old click bind first
        $("#savePrefixButton").unbind('click');
        $("#savePrefixButton").click(function() {
            // hide dialog
            $("#prefixPopup").modal("hide");

            // save arguments
            selectedModule.prefixes = [];  // empty list
            for (var i=0; i< $('#prefixPopupBody > table > tbody > tr').not(":first").length; i++) {
                var prefix = {};
                prefix.execId = $($('#prefixPopupBody > table > tbody > tr')[i+1].children[0]).text();
                prefix.prefix = $($('#prefixPopupBody > table > tbody > tr')[i+1].children[1].children[0]).val();
                selectedModule.prefixes.push(prefix);
            }
            updateStatus("Saved prefixes.");
        });
    });

    // for parameters
    for (var i=0; i < selectedModule.executables.length; i++) {
        var exe = selectedModule.executables[i];
        for (var j=0; j < exe.params.length; j++) {
            var param = exe.params[j];
            var default_value = param.default;
            for (set_param in selectedModule.params) {
                if (selectedModule.params[set_param].name == param.name) {
                    default_value = selectedModule.params[set_param].value;
                }
            }
            tbody.append('<tr><th>'+ param.name +'</th><td><input class="form-control" type="text" value="'+default_value+'" data-targetmoduleuid="'+selectedModuleUid+'"/></td></tr>');
        }
    }
    if (tbody.html() == '') {
        tbody.append('<tr><td colspan="2"><em>No params</em></td></tr>');
    }

    // save the params in the modulelist
    $(document).on('blur', 'input.form-control', function (event) {
        var targetModule = getModuleByUWindowId($(this).data('targetmoduleuid'));

        var param = {};  // premake object
        param.name = $(this).parent().siblings().text(); // set name
        param.value = $(this).val();  // set value

        // check if the parameter has been set before
        var parameterWasSetBefore = false;
        for (var i=0; i<targetModule.params.length; i++) {
            var setParam = targetModule.params[i];
            if (setParam.name == param.name) {
                // update value if it has been set
                setParam.value = param.value;
                parameterWasSetBefore = true;
            }
        }
        if (parameterWasSetBefore == false) {
            // add if it hasnt been set
            targetModule.params.push(param);
        }
    });

    // for machine selection
    $('#machineSelectButton').click(function() {
        var dropdownOptions = '<option value="">Default (localhost)</option>';
        $.each(machines, function(){
           dropdownOptions += '<option value='+this.name+'>'+this.name+'</option>';
        });

        saveState();
        // clear list of executables
        $("#machineSelectPopupBody > table > tbody >tr").not(":first").remove();

        // make list of executables
        for(var i=0; i<selectedModule.executables.length; i++){
            var texec = selectedModule.executables[i];
            var selectedMachine;
            if (typeof selectedModule.selectedMachines[i] === "undefined") {
                selectedMachine = false;
            }
            else {
                selectedMachine = selectedModule.selectedMachines[i].machineName;
            }

            var dropdownOptions = '<option value="">Default (local)</option>';
            $.each(machines, function(){
                dropdownOptions += '<option';
                if(this.name == selectedMachine)
                {
                    dropdownOptions += ' selected="selected"';
                }
                dropdownOptions += ' value='+this.name+'>'+this.name+'</option>';
            });

            var tc = "<tr><td>"+texec.id+'</td><td><select data-exec="'+texec.id+'" class="form-control">' + dropdownOptions + "></select></td></tr>";
            $("#machineSelectPopupBody > table > tbody").append(tc);
        }

        // Remove old click bind first
        $("#saveMachineSelectButton").unbind('click');
        $("#saveMachineSelectButton").click(function() {
            // hide dialog
            $("#machineSelectPopup").modal("hide");

            // save arguments
            selectedModule.selectedMachines = [];  // empty list
            for (var i=0; i< $('#machineSelectPopupBody > table > tbody > tr').not(":first").length; i++) {
                var selection = {};
                selection.execId = $($('#machineSelectPopupBody > table > tbody > tr > td > select')[i]).data('exec');
                selection.machineName = $($('#machineSelectPopupBody > table > tbody > tr > td > select')[i]).val();
                selectedModule.selectedMachines.push(selection);
            }

            updateStatus("Saved machine selection.");
        });
    });

}

/****************************************
 * this is where the jsPlumb starts
 ***************************************/
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

jsPlumb.ready(function() {
    jsPlumbInstance = jsPlumb.getInstance({
        // default drag options
        DragOptions: { cursor: 'pointer', zIndex: 2000 },
        // the overlays to decorate each connection with.  note that the label overlay uses a function to generate the label text; in this
        // case it returns the 'labelText' member that we set on each connection in the 'init' method below.
        ConnectionOverlays: [
            [ "Arrow", { location: 1 } ],
        ],
        Container: "project-container"
    });

    // suspend drawing and initialise.
    jsPlumbInstance.batch(function () {

        // listen for new connections; initialise them the same way we initialise the connections at startup.
        jsPlumbInstance.bind("beforeDrop", function (connInfo) {
            saveState();
            sourceType = connInfo.connection.endpoints[0].getParameter("type");
            targetType = connInfo.dropEndpoint.getParameter("type");

            if (sourceType != targetType) {
                if (sourceType == "ANY") {
                    return true;
                }
                else if (targetType == "ANY") {
                    return true;
                }
                else{
                    updateStatus("Not able to connect endpoints of different types");
                    return false;
                }
            }

            return true;
        });

        jsPlumbInstance.bind("beforeDetach", function(e){
            saveState();
        });

        jsPlumbInstance.bind("connectionDragStop", function(connection) {
            jsPlumbInstance.selectEndpoints().each(function (endpoint) {
                endpoint.removeClass('validDropPoint');
                endpoint.removeClass('invalidDropPoint');
            });
        });

        jsPlumbInstance.bind("connectionDrag", function (connection) {
            var sourceType = connection.endpoints[0].getParameter("type");
            var connections = jsPlumbInstance.selectEndpoints().each(function(endpoint) {
                // Color code all target endpoints based on the source type
                if (endpoint.isTarget) {
                    if (sourceType == "ANY") {
                        endpoint.addClass('validDropPoint');
                    } else if (endpoint.getParameter('type') == sourceType) {
                        endpoint.addClass('validDropPoint');
                    } else if (endpoint.getParameter('type') == "ANY") {
                        endpoint.addClass('validDropPoint')
                    } else {
                        endpoint.addClass('invalidDropPoint');
                    }
                }
            });
        });

        // make all the window divs draggable
        jsPlumbInstance.draggable($(".project-container .window"), { grid: [20, 20], start: saveState, containment: "parent"});

        /*
         Connection click handler...
         */
        jsPlumbInstance.bind("click", function (conn, originalEvent) {
            if (originalEvent.ctrlKey) { // delete the connection on ctrl click
                jsPlumbInstance.detach(conn);
            }
        });
    });
});
/****************************************
 * this is where the jsPlumb ends
 ***************************************/

/**
 * Add several input markers (endpoints) to a window (module)
 * @param moduleId id of the module to add inputs to
 * @param inputs to add to it
 */
function addInputsToWindow(moduleId, inputs) {
    // creat the enpoint class
    var targetEndpoint = {
        endpoint: "Dot",
        paintStyle: { fillStyle: "#7AB02C", radius: 11 },
        hoverPaintStyle: endpointHoverStyle,
        dropOptions: { hoverClass: "hover", activeClass: "active" },
        isTarget: true
    };

    // add enpoints for every input
    for (var i = 0; i < inputs.length; i++) {
        var anchorId = moduleId + "input" + i.toString;
        var pos = [0, ((i+1)/(inputs.length +1)), -1, 0];
        jsPlumbInstance.addEndpoint(moduleId, targetEndpoint, {
            uuid: moduleId + "/" + inputs[i].name,
            anchor: pos,
            parameters: {
                type: inputs[i].message_type,
                name: inputs[i].name
            },
            maxConnections: 10,
            overlays: [
                [ "Label", {
                    location: [-1.5, 0.5],
                    label: inputs[i].name,
                    cssClass: "endpoint-label"
                } ]
            ],
        });
    }
}

/**
 * Add several output markers (endpoints) to a window (module)
 * @param moduleId id of the module to add outputs to
 * @param outputs outputs to add to it
 */
function addOutputsToWindow(moduleId, outputs) {
    // create the endpoint
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

    // add an endpoint for every output
    for (var i = 0; i < outputs.length; i++) {
        var anchorId = moduleId + "output" + i.toString;
        var pos = [1, ((i+1)/(outputs.length +1)), 1, 0];
        jsPlumbInstance.addEndpoint(moduleId, sourceEndpoint, {
            uuid: moduleId + "/" + outputs[i].name,
            anchor: pos,
            parameters: {
                type: outputs[i].message_type,
                name: outputs[i].name
            },
            maxConnections: 10,
            overlays: [
                [ "Label", {
                    location: [-1.5, 0.5],
                    label: outputs[i].name,
                    cssClass: "endpoint-label"
                } ]
            ]
        });
    }
}
/**
 * This function collects some data when you start dragging a module
 * first the state of the window is saved, the module id is set to be 
 * available anywhere, and some feedback is given
 * @param ev the dragging event
 */
var current_dragging_module_id = null;
function startDrag(ev) {
    saveState();
    ev = ev.originalEvent;
    updateStatus("Dragging module: " + ev.target.id + ".");
    current_dragging_module_id = ev.target.id;
}

/**
 * Prevents the default drop function so that we can 
 * more easily access our own
 */
function allowDrop(ev) {
    ev.preventDefault();
}

/**
 * Drops a module onto the canvas
 * Check if you are indeed dragging a module. If so,
 * add it to the canvas
 * @parem ev Drop event
 */
function dropModule(ev) {
    // first check if we started dragging a module
    if (current_dragging_module_id == null) {
        return null;
    }

    updateStatus("Dropped a module to the project-container.");
    ev.preventDefault();

    saveState();
    addModuleToContainer(current_dragging_module_id, ev.pageX, ev.pageY);
    current_dragging_module_id = null;
}

/**
 * Adds a module to the container (the canvas)
 * after which it may be connected to others
 * @param modId module id of the module to add to the canvas
 * @param _x horizontal position to position the module
 * @param _y vertical positon to position the module
 * @param uModId Unique id to give to this module in the canvas
 */
function addModuleToContainer(modId, _x, _y, uModId) {
    var modToAdd;
    if (typeof(uModId) == 'undefined' || uModId == null)
    {
        // uModId is not set, generate.
        uModId = modId + guid();  // generate unique id
        modToAdd = getModuleById(modId);
    }
    else
    {
        // uModId is set, so find original module name
        strippedModId = uModId.substring(0, uModId.length - 5);
        modToAdd = getModuleById(strippedModId);
    }

    // add to inCanvasArray
    modToAdd.uWindowId = uModId;
    modToAdd.params = [];  // premake list for parameters
    modToAdd.args = [];  // placeholder for command line arguments
    modToAdd.prefixes = [];  // placeholder for launch-prefixes
    modToAdd.selectedMachines = [];

    for (ex_index in modToAdd.executables)
    {
        var ex = modToAdd.executables[ex_index];
        modToAdd.args.push({'execId': ex.id, 'cmd': ex.args});
    }

    modulesInCanvas.push(modToAdd);
    
    $(".project-container").append('<div class="window" id="'+uModId+'"><span class="window_label">'+modToAdd.meta.name+'</span></div>');

    // start module at correct position
    var width = 100;
    var height = 20 + Math.max(modToAdd.inputs.length, modToAdd.outputs.length) * 20;
    var X = Math.floor((_x - 0.5*width) / 20) * 20;
    var Y = Math.floor((_y - 0.5*height) / 20) * 20;

    $(".project-container #"+uModId).offset({
        top : Y,
        left: X
    }).width(width)
        .height(height);

    // make draggable
    jsPlumbInstance.draggable($(".project-container .window"), { grid: [20, 20], start: saveState, containment: "parent" });

    // connections
    addInputsToWindow(uModId, modToAdd.inputs);
    addOutputsToWindow(uModId, modToAdd.outputs);

    updateStatus("Module added succesfully.");
}

/**
 * Returns a module from the available module list based 
 * on only its id
 * @param Id the id of the module to return
 * @returns The module you requested
 */
function getModuleById(Id) {
    for (var i=0; i <modules.length; i++) {
        if (modules[i].id == Id) {
            // this returns a copy of the object, so we can modify it without mofifying our initial list
            return $.extend(true, {}, modules[i]); 
        }
    }
}

/**
 * returns a module from the canvas, based on its unique id
 * @param uId unique id of the module to return
 * @returns the module you requested
 */
function getModuleByUWindowId(uId) {
    for (var i=0; i<modulesInCanvas.length; i++) {
        if (modulesInCanvas[i].uWindowId == uId) {
            return modulesInCanvas[i];
        }
    }
}

/**
 * Generate a unique id for a module.
 * the id is saved in a list, so not even accidentally id's can be the same
 * @returns a unique id
 */
function guid() {
    // generate until a not-used number has been found    
    do {
        var generatedId = "_" + ("" + (Math.random()*Math.pow(36,4) << 0).toString(36)).slice(-4);
    }
    while(generatedGUIDs.indexOf(generatedId) > -1);

    // save number in list
    generatedGUIDs.push(generatedId);

    return generatedId;
}

/**
 * deletes a module (window) from the canvas (module container)
 * @param uId unique id of the module to delete
 */
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
    updateStatus("Module removed!");
}

/**
 * get the configuration you build in the canvas as a javascript object
 * @returns the config object
 */
function getConfig() {
    updateStatus("Getting config...");

    var config = {};
    config.modsToAdd = [];
    config.connectsToAdd = [];
    $(".project-container").children().each(function() {
        uId = this.id;  // unique id for this element
        var mod = {};  // initialize module object
        
        // save all modules (only use role, type and id)
        var tempmod = getModuleByUWindowId(uId);  // get infor from unique id   
        if (tempmod == undefined) {
            console.log("Module with uId: '"+uId+"' could not be found.. :(");
        }

        mod.id = uId;  // save id
        mod.type = tempmod.id;  // save type
        mod.role = tempmod.type;  // save unique id
        mod.params = [];
        // add all parameters and values
        for (var i=0; i<tempmod.executables.length; i++) {
            var exec_i = tempmod.executables[i];
            for (var j = 0; j < tempmod.params.length; j++) {
                var param = tempmod.params[j];
                var pval = param.default;
                if (param.value != null) {
                    pval = param.value;
                }
                mod.params.push({'name': param.name, 'value': pval});
            }
        }

        // arguments
        mod.args = [];
        for (var i=0; i<tempmod.args.length; i++) {
            mod.args.push(tempmod.args[i]);
        }

        // launch prefixes
        mod.prefixes = [];
        for (var i=0; i<tempmod.prefixes.length; i++) {
            mod.prefixes.push(tempmod.prefixes[i]);
        }

        // machine selections
        mod.selectedMachines = [];
        for (var i=0; i<tempmod.selectedMachines.length; i++) {
            mod.selectedMachines.push(tempmod.selectedMachines[i]);
        }

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
                        console.log('Found an endpoint that is not a source, nor a target');
                        alert('error, please check console.log');
                    }                
                }    

                // save to connectionlist
                config.connectsToAdd.push(connectionToAdd);
            }
            
        }
    });

    updateStatus("done.");
    return config;
}

/**
 * Get the xml document representation of a config object
 * @param config the config object
 * @returns the requested xml as string
 */
function getConfigXML(config) {
    updateStatus("Generating XML...");
    // create config XML 
    var configXML = document.createElement("configuration");
    configXML.setAttribute("id", "VIKI-imported-config");

    $.each(machines, function(){
        var machineXML = document.createElement('machine');
        machineXML.setAttribute('name', this.name);
        machineXML.setAttribute('hostname', this.hostname);
        machineXML.setAttribute('username', this.username);
        machineXML.setAttribute('password', this.password);
        configXML.appendChild(machineXML);
    });

    // add modules to the config XML
    for (var i=0; i<config.modsToAdd.length; i++) {
        var tempMod = config.modsToAdd[i];
        var modXML = document.createElement(tempMod.role);
        modXML.setAttribute("type", tempMod.type);
        modXML.setAttribute("id", tempMod.id);

        // parameters
        for (var j=0; j < config.modsToAdd[i].params.length; j++) {
            var param = config.modsToAdd[i].params[j];
            var paramXML = document.createElement('param');
            paramXML.setAttribute('name', param.name);
            paramXML.setAttribute('value', param.value);

            /* UGLY HACK WARNING
            This works, but I don't know why...
            The document.createElement does not support forcing a close tag,
            so actually we should write the generation of xml ourselves.
            Here I add an element to the parameter XML, so it will close
            */

            var subX = document.createElement('x');
            paramXML.appendChild(subX);

            modXML.appendChild(paramXML);
        }

        // arguments
        for (var j=0; j<config.modsToAdd[i].args.length; j++) {
            var arg = config.modsToAdd[i].args[j];
            var argXML = document.createElement('arg');
            argXML.setAttribute('exec_id',arg.execId);
            argXML.setAttribute('argument',arg.cmd);
            modXML.appendChild(argXML);
        }

        // launch prefixes
        for (var j=0; j<config.modsToAdd[i].prefixes.length; j++) {
            var prefix = config.modsToAdd[i].prefixes[j];
            var prefixXML = document.createElement('launch-prefix');
            prefixXML.setAttribute('exec_id',prefix.execId);
            prefixXML.setAttribute('prefix',prefix.prefix);
            modXML.appendChild(prefixXML);
        }

        // selected machines
        for (var j=0; j<config.modsToAdd[i].selectedMachines.length; j++) {
            var selection = config.modsToAdd[i].selectedMachines[j];
            if(selection.machineName != '')
            {
                var selectionXML = document.createElement('selected-machine');
                selectionXML.setAttribute('exec_id',selection.execId);
                selectionXML.setAttribute('machine_name',selection.machineName);
                modXML.appendChild(selectionXML);
            }
        }

        // add module to XML
        configXML.appendChild(modXML);
    }

    // add connects to the config XML
    for (var i=0; i<config.connectsToAdd.length; i++){
        var connectXML = document.createElement("connect");
        connectXML.setAttribute("publisher", config.connectsToAdd[i].pub);
        connectXML.setAttribute("listener", config.connectsToAdd[i].sub);
        configXML.appendChild(connectXML);
    }
        
    updateStatus("Done!");
    // return
    return configXML.outerHTML;
    // TODO: Should the line below be there?
    //send(JSON.stringify({name: "vikiMake", value: configXML.outerHTML}));
}

/**
 * Saves the localhostname
 */
function setLocalHostName(hostName) {
    localHostName = hostName;
    send(JSON.stringify({name: 'vikiSetMasterUri', value: hostName}));
    $("#localHostName").html(localHostName);
}

/**
 * populates the list of machines with machines currently known
 * @param machines currently known machines
 */
function syncMachineList(machines) {
    $("#machineList").find("tr:gt(0)").remove();
    $.each(machines,function(){
        $("#machineList").append(
            '<tr><td>'+this.name+'</td><td>'+this.hostname+'</td><td>'+this.username+'</td><td>'+this.password+'</td>' +
            '<td>' +
            '<a data-key="'+this.name+'" href class="btn btn-default editMachine"><i class="glyphicon glyphicon-edit"></i></a>' +
            '<a data-key="'+this.name+'" href class="btn btn-default deleteMachine"><i class="glyphicon glyphicon-remove"></i></a>' +
            '</td>' +
            '</tr>'
        );
    });
}