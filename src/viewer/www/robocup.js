// Field geometry
var field = {
    length: 9.0,
    width: 6.0,
    borders: 0.5,
    centerCircleRadius: 0.5,
    penaltyAreaWidth: 2,
    penaltyAreaDepth: 1,
    goalWidth: 1,
    goalDepth: 0.18,
    goalThickness: 0.023,
    linesThickness: 0.02,
    boundaryWidth: 0.25
};

// Blue on positive ?
var bluePositive = true;
var yellowName = 'yellow';
var blueName = 'blue';

// Robots
var robots = {
    'yellow': {},
    'blue': {}
};

// Our team color
var ourColor = 'blue';

// Is it simulation ?
var simulation = false;

// Ball
var ball = [0, 0];

// Strategies
var strategies = [];

function normalizeTheta(theta)
{
    return theta - (2*Math.PI) * Math.floor((theta + Math.PI) / (2*Math.PI));
}

function robotById(id, color)
{
    if (typeof(color) == 'undefined') {
        color = ourColor;
    }

    if (color in robots && id in robots[color]) {
        return robots[color][id];
    } else {
        return null;
    }
}



function Viewer()
{
    this.mouseMove = function(evt)
    {
        this.mousePos = [evt.clientX, evt.clientY];
    };

    // Compute the mouse position un meter
    this.mousePosMeters = function()
    {
        var left = (this.width-(field.length/this.ratio))/2;
        var top = (this.height-(field.width/this.ratio))/2;
        var X = ((this.mousePos[0]-left)/this.ratio);
        var Y = -((this.mousePos[1]-top)/this.ratio);
        X -= this.viewOffset[0];
        Y -= this.viewOffset[1];

        if (this.reversed) {
            X = -X;
            Y = -Y;
        }

        return [X, Y];
    };

    // Mouse angle since begining of the drag
    this.mouseAngle = function(robot)
    {
        if (this.mousePos) {
            var pos = this.mousePosMeters();
            var dx = pos[0] - robot.x;
            var dy = pos[1] - robot.y;

            return Math.atan2(dy, dx);
        }

        return 0;
    };

    // Resetting the ratio to fit the canvas zone
    this.resetRatio = function()
    {
        var ratio2 = this.height/(field.width + 2*field.borders);
        var ratio1 = this.width/(field.length + 2*field.borders);
        if (ratio2 < ratio1) this.ratio = ratio2;
        else this.ratio = ratio1;
        this.viewOffset = [0, 0];
    };

    this.grColor = function(color)
    {
        if (!this.greenred) {
            if (color == 'yellow') {
                return '#ede370';
            } else {
                return '#70d1ed';
            }
        } else {
            if (color == ourColor) {
                return '#83e568';
            } else {
                return '#e56868';
            }
        }
    };

    this.addText = function(message, x, y, options)
    {
        if (typeof(options) == 'undefined') {
            options = {};
        }
        options.text = message;
        options.x = x;
        options.y = y;

        this.texts.push(options);
    };

    this.update = function()
    {
        var ctx = this.ctx;

        this.texts = [];

        // Retrieving zone sizes
        this.width = this.zone.width();
        this.height = this.zone.height();
        this.container.width = this.width;
        this.container.height = this.height;

        // ctx.clearRect(0, 0, 100, 100);
        ctx.clearRect(0, 0, this.width, this.height);

        // Translating to the middle, rescaling for meters
        ctx.save();
        if (this.ratio == null) {
            this.resetRatio();
        }

        // Translating to the center
        ctx.translate(this.width/2, this.height/2);

        // Rescaling using ratio
        ctx.scale(this.ratio, -this.ratio);

        // Translating to the view offset
        ctx.translate(this.viewOffset[0], this.viewOffset[1]);

        // Reverse
        if (this.reversed) {
            this.sign = -1;
        } else {
            this.sign = 1;
        }

        // Drawing the field
        this.drawField();

        // Drawing robots
        this.drawRobots();

        if (this.mousePos) {
            var robot = null;
            var pos = this.mousePosMeters();

            // Dragging a robot
            if (this.draggingRobot) {
                robot = robotById(this.draggingRobot[0], this.draggingRobot[1]);
                robot = JSON.parse(JSON.stringify(robot));
                robot.ghost = true;
                robot.x = pos[0];
                robot.y = pos[1];
                this.drawRobot(robot);
            }

            // Rotating a robot
            if (this.rotatingRobot) {
                robot = robotById(this.rotatingRobot[0], this.rotatingRobot[1]);
                robot = JSON.parse(JSON.stringify(robot));

                var angle = this.mouseAngle(robot);

                robot.ghost = true;
                robot.orientation = angle;
                this.drawRobot(robot);
            }
        }

        // Drawing the ball
        this.drawBall();

        // Drawing annotations
        this.drawAnnotations();

        ctx.restore();

        // Drawing texts
        ctx.fillStyle = '#aaa';
        ctx.font = '12pt sans';
        if (this.mousePos) {
            var mpos = this.mousePosMeters();

            ctx.fillText("X: "+mpos[0].toFixed(2)+"m, Y: "+mpos[1].toFixed(2)+"m", 10, 20);
            ctx.fillStyle = this.grColor('yellow');
            ctx.fillText('Yellow: '+yellowName, 10, 40);
            ctx.fillStyle = this.grColor('blue');
            ctx.fillText('Blue: '+blueName, 10, 60);

            // Moving the view offset
            if (this.dragging) {
                var dx = (this.mousePos[0] - this.dragBegin[0])/this.ratio;
                var dy = -(this.mousePos[1]- this.dragBegin[1])/this.ratio;
                this.viewOffset = [this.startOffset[0] + dx, this.startOffset[1] + dy];
            }
        }

        // Drawing text messages
        ctx.save();
        ctx.translate(this.width/2, this.height/2);
        ctx.translate(this.ratio*this.viewOffset[0], -this.ratio*this.viewOffset[1]);

        for (var k in this.texts) {
            ctx.save();
            var text = this.texts[k];
            ctx.font = (this.ratio/10)+'pt sans';
            if ('color' in text) {
                ctx.fillStyle = text.color;
            }
            ctx.fillText(text.text, text.x*this.ratio, text.y*this.ratio);
            ctx.restore();
        }
        ctx.restore();
    };

    this.drawField = function()
    {
        var ctx = this.ctx;

        // Border lines
        ctx.strokeStyle = '#aaa';
        ctx.lineWidth = field.linesThickness;
        ctx.strokeRect(-field.length/2, -field.width/2, field.length, field.width);

        // Boundary lines
        ctx.save();
        ctx.strokeStyle = 'black';
        ctx.lineWidth = 2*field.linesThickness;
        var margin = field.boundaryWidth+field.goalDepth;
        ctx.strokeRect(-(field.length/2)-margin, -(field.width/2)-margin, field.length+2*margin, field.width+2*margin);
        ctx.restore();

        // Lines
        ctx.beginPath();
        ctx.moveTo(0, field.width/2);
        ctx.lineTo(0, -field.width/2);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(-field.length/2, 0);
        ctx.lineTo(field.length/2, 0);
        ctx.stroke();

        // Center circle
        ctx.beginPath();
        ctx.arc(0, 0, field.centerCircleRadius, 0, Math.PI*2);
        ctx.stroke();

        function drawGoals(ctx, color, teamName) {
            // Goals zone
            ctx.strokeStyle = '#aaa';
            ctx.lineWidth = field.linesThickness;
            ctx.beginPath();
            ctx.moveTo(-field.length/2, -field.penaltyAreaWidth/2);
            ctx.lineTo(-field.length/2 + field.penaltyAreaDepth, -field.penaltyAreaWidth/2);
            ctx.lineTo(-field.length/2 + field.penaltyAreaDepth, field.penaltyAreaWidth/2);
            ctx.lineTo(-field.length/2, field.penaltyAreaWidth/2);
            ctx.stroke();

            // Goals
            ctx.strokeStyle = color;
            ctx.lineWidth = field.goalThickness;
            ctx.beginPath();
            ctx.moveTo(-field.length/2, field.goalWidth/2);
            ctx.lineTo(-field.length/2 - field.goalDepth, field.goalWidth/2);
            ctx.lineTo(-field.length/2 - field.goalDepth, -field.goalWidth/2);
            ctx.lineTo(-field.length/2, -field.goalWidth/2);
            ctx.stroke();
        }

        // Drawing (symmetrically) goals
        ctx.save();
        if (!bluePositive) ctx.scale(-1, 1);
        drawGoals(ctx, this.reversed ? this.grColor('blue') : this.grColor('yellow'));
        ctx.restore();

        ctx.save();
        if (bluePositive) ctx.scale(-1, 1);
        drawGoals(ctx, this.reversed ? this.grColor('yellow') : this.grColor('blue'),
            bluePositive ? blueName : yellowName);
        ctx.restore();
    };

    this.drawRobots = function()
    {
        var ctx = this.ctx;
        var over = this.overRobot();

        for (var team in robots) {
            for (var k in robots[team]) {
                var robot = robots[team][k];
                this.drawRobot(robot, over == robot);
            }
        }
    };

    this.drawRobot = function(robot, extra)
    {
        var ctx = this.ctx;
        var front = 0.75;

        if (!('ghost' in robot) && this.rotatingRobot && this.rotatingRobot[0] == robot.id &&
            this.rotatingRobot[1] == robot.team) {
            return;
        }

        ctx.save();
        ctx.globalAlpha = robot.present ? 1 : 0.5;
        if ('ghost' in robot) {
            ctx.globalAlpha = 0.5;
        }

        ctx.save();
        if (this.reversed) {
            ctx.scale(-1, -1);
        }

        ctx.beginPath();
        ctx.strokeStyle = '#aaa';
        ctx.fillStyle = this.grColor(robot.team);
        if(robot.active){
            //ctx.arc(robot.x_odom, robot.y_odom, 0.1, (robot.t_odom/1000)+front, (robot.t_odom/1000)+Math.PI*2-front);
            ctx.arc(robot.x, robot.y, 0.1, robot.orientation+front, robot.orientation+Math.PI*2-front); 

        }
        else{
            ctx.arc(robot.x, robot.y, 0.1, robot.orientation+front, robot.orientation+Math.PI*2-front);
        }
        
        if (!robot.present) {
            ctx.stroke();
        }
        ctx.fill();

        ctx.restore();
        var color = robot.present ? '#333' : 'white';

        this.addText(''+robot.id,this.sign*robot.x-0.05, -this.sign*robot.y+0.055, {color: color});

        if (typeof(extra) != 'undefined' && extra) {
            ctx.fillStyle = '#aaa';
            this.addText(robot.team+' #'+robot.id, this.sign*robot.x+0.15, -this.sign*robot.y-0.15, {color: '#aaa'});
            this.addText('x:'+this.sign*robot.x.toFixed(2)+'m', this.sign*robot.x+0.15, -this.sign*robot.y, {color: '#aaa'});
            this.addText('y:'+this.sign*robot.y.toFixed(2)+'m', this.sign*robot.x+0.15, -this.sign*robot.y+0.15, {color: '#aaa'});
            this.addText('t:'+(180*robot.orientation/Math.PI).toFixed(1)+'°', this.sign*robot.x+0.15, -this.sign*robot.y+0.3, {color: '#aaa'});
        }

        if ("ir" in robot && robot.ir && robot.com) {
            ctx.beginPath();
            ctx.globalAlpha = 0.5;
            ctx.strokeStyle = '#f51a90';
            ctx.lineWidth = 0.01;
            ctx.arc(robot.x + Math.cos(robot.orientation)*0.085,
            robot.y+Math.sin(robot.orientation)*0.085,
            0.03, -Math.PI/2, Math.PI/2);
            ctx.stroke();
            ctx.beginPath();
            ctx.arc(robot.x + Math.cos(robot.orientation)*0.085,
            robot.y+Math.sin(robot.orientation)*0.085,
            0.06, -Math.PI/2, Math.PI/2);
            ctx.stroke();
        }

        ctx.restore();
    };

    this.drawBall = function()
    {
        var ctx = this.ctx;

        var pos = JSON.parse(JSON.stringify(ball));
        if (this.draggingBall && this.mousePos) {
            pos = this.mousePosMeters();
        }

        if (this.reversed) {
            pos[0] = -pos[0];
            pos[1] = -pos[1];
        }

        ctx.beginPath();
        ctx.fillStyle = 'orange';
        ctx.arc(pos[0], pos[1], 0.043, 0, Math.PI*2);
        ctx.fill();

        if (this.overBall()) {
            this.addText('ball', pos[0]+0.15, -pos[1]-0.15, {color: 'orange'});
            this.addText('x:'+pos[0].toFixed(2)+'m', pos[0]+0.15, -pos[1], {color: 'orange'});
            this.addText('y:'+pos[1].toFixed(2)+'m', pos[0]+0.15, -pos[1]+0.15, {color: 'orange'});
        }
    };

    this.drawAnnotations = function()
    {
        var ctx = this.ctx;
        var annotations = JSON.parse(api.getAnnotations());

        for (var k in annotations) {
            var annotation = annotations[k];
            ctx.fillStyle = annotation.color;
            ctx.strokeStyle = annotation.color;
            if (annotation.dashed) {
                ctx.setLineDash([0.02, 0.02]);
            } else {
                ctx.setLineDash([]);
            }

            switch (annotation.type) {
                case "circle": {
                    ctx.beginPath();
                    ctx.arc(annotation.x, annotation.y, annotation.r, 0, Math.PI*2);
                    ctx.stroke();
                }
                break;
                case "arrow": {
                    ctx.save();
                    ctx.beginPath();
                    ctx.moveTo(annotation.x, annotation.y);
                    ctx.lineTo(annotation.toX, annotation.toY);

                    ctx.moveTo(annotation.toX, annotation.toY);
                    var d = 0.12;
                    var alpha = Math.atan2(annotation.y-annotation.toY, annotation.x-annotation.toX);
                    ctx.lineTo(annotation.toX + Math.cos(alpha+0.4)*d,
                    annotation.toY + Math.sin(alpha+0.4)*d);

                    ctx.moveTo(annotation.toX, annotation.toY);
                    ctx.lineTo(annotation.toX + Math.cos(alpha-0.4)*d,
                    annotation.toY + Math.sin(alpha-0.4)*d);

                    ctx.stroke();
                    ctx.restore();
                }
                break;
                case "cross": {
                    ctx.beginPath();
                    ctx.moveTo(annotation.x - 0.1, annotation.y - 0.1);
                    ctx.lineTo(annotation.x + 0.1, annotation.y + 0.1);
                    ctx.moveTo(annotation.x - 0.1, annotation.y + 0.1);
                    ctx.lineTo(annotation.x + 0.1, annotation.y - 0.1);
                    ctx.stroke();
                }
                break;
                case "text": {
                    this.addText(
                        annotation.text,
                        annotation.x,
                        -annotation.y,
                        { color: '#FFF' }
                    );
                }
                break;
            }
        }
    };

    this.overBall = function()
    {
        if (this.mousePos) {
            var xy = this.mousePosMeters();
            var dist = Math.sqrt(Math.pow(ball[0]-xy[0], 2) + Math.pow(ball[1]-xy[1], 2));

            return dist < 0.1;
        }

        return false;
    };

    this.overRobot = function()
    {
        if (this.mousePos) {
            var xy = this.mousePosMeters();
            for (var team in robots) {
                for (var k in robots[team]) {
                    var robot = robots[team][k];
                    var dist = Math.sqrt(Math.pow(robot.x-xy[0], 2) + Math.pow(robot.y-xy[1], 2));
                    if (dist < 0.1) {
                        return robot;
                    }
                }
            }
        }

        return null;
    };

    // Mouse is down
    this.mouseDown = function(evt)
    {
        var over = false;
        if (this.mousePos) {
            this.dragBegin = this.mousePos;

            if (evt.originalEvent.which == 1) {
                if (this.overBall()) {
                    this.draggingBall = true;
                } else {
                    over = this.overRobot();
                    if (over) {
                        this.draggingRobot = [over.id, over.team];
                    } else if (!this.dragLock) {
                        // Mouse3 to drag the view
                        this.dragging = true;
                        this.startOffset = this.viewOffset;
                    }
                }
            }

            if (evt.originalEvent.which == 2) {
                over = this.overRobot();

                if (over) {
                    this.rotatingRobot = [over.id, over.team];
                }
            }
        }
    };

    // Stopping the dragging
    this.mouseUp = function(evt)
    {
        var robot = null;
        var pos = null;

        // Dragging the robot
        if (this.draggingRobot && this.mousePos) {
            robot = robotById(this.draggingRobot[0], this.draggingRobot[1]);

            if (robot) {
                deltaPixels = Math.sqrt(Math.pow(this.mousePos[0]-this.dragBegin[0], 2) +
                Math.pow(this.mousePos[1]-this.dragBegin[1], 2));

                if (deltaPixels > 10) {
                    pos = this.mousePosMeters();

                    api.moveRobot(this.draggingRobot[1] == 'yellow',
                        this.draggingRobot[0], pos[0], pos[1], robot.orientation);
                } else {
                    if (robot.team == ourColor) {
                        this.manager.robotClicked(robot);
                    }
                }
            }
        }

        // Rotating a robot
        if (this.rotatingRobot && this.mousePos) {
            robot = robotById(this.rotatingRobot[0], this.rotatingRobot[1]);
            if (robot) {
                var angle = this.mouseAngle(robot);

                api.moveRobot(this.rotatingRobot[1] == 'yellow',
                    this.rotatingRobot[0], robot.x, robot.y, angle);
            }
        }

        // Dragging the ball
        if (this.draggingBall && this.mousePos) {
            pos = this.mousePosMeters();

            api.moveBall(pos[0], pos[1]);
        }

        this.dragging = false;
        this.draggingRobot = null;
        this.rotatingRobot = null;
        this.draggingBall = false;
    };

    // Spinning the mouse wheel
    this.mouseWheel = function(evt)
    {
        if (!this.dragLock) {
            this.ratio += 15*evt.deltaY;
            if (this.ratio < 0) {
                this.ratio = 1;
            }
            evt.preventDefault();
        }
    };

    // A key was pressed on the keyboard
    this.keyPress = function(evt)
    {
        if (evt.which == 27) {
            this.resetRatio();
        }
        if (evt.which == 178) {
            $('.reverse-view').click();
            evt.preventDefault();
        }
    };

    this.zone = $('.field-zone');
    this.container = null;
    this.ctx = null;
    this.mousePos = null;
    this.ratio = null;
    this.dragging = false;
    this.draggingRobot = null;
    this.draggingBall = false;
    this.rotatingRobot = null;
    this.dragLock = false;
    this.dragBegin = null;
    this.viewOffset = [0, 0];
    this.startOffset = [0, 0];
    this.reversed = false;
    this.greenred = false;

    this.container = document.getElementById('field');
    this.ctx = this.container.getContext('2d');

    var self = this;

    $('#field').mousemove(function(evt) { self.mouseMove(evt); });
    $('#field').mousedown(function(evt) { self.mouseDown(evt); });
    $('#field').mouseup(function(evt) { self.mouseUp(evt); });
    $('#field').mousewheel(function(evt) { self.mouseWheel(evt); });

    document.addEventListener('keypress', function(evt) { self.keyPress(evt); });
}

function Manager(viewer)
{
    this.field = field;

    this.updateApi = function()
    {
        // Getting field dimensions
        var fieldStatus = JSON.parse(api.fieldStatus());
        if (fieldStatus.present) {
            this.field.length = fieldStatus.length;
            this.field.width = fieldStatus.width;
            this.field.goalWidth = fieldStatus.goalWidth;
            this.field.goalDepth = fieldStatus.goalDepth;
            this.field.boundaryWidth = fieldStatus.boundaryWidth;
            this.field.penaltyAreaWidth = fieldStatus.penaltyAreaWidth;
            this.field.penaltyAreaDepth = fieldStatus.penaltyAreaDepth;
        }

        // Getting robots position from the API and update it
        var robotsStatus = JSON.parse(api.robotsStatus());
        for (var k in robotsStatus) {
            var status = robotsStatus[k];

            if (!(status.id in robots[status.team])) {
                // Creating robot entry
                status.spin = false;
                robots[status.team][status.id] = status;

                if (status.team == ourColor && simulation) {
                    api.robotCommand(status.id, 0.0, 0.0, 0.0);
                }
            } else {
                for (var field in status) {
                    // Updating fields
                    robots[status.team][status.id][field] = status[field];
                }
            }
        }

        // Getting ball infos
        var ballStatus = JSON.parse(api.ballStatus());
        ball = [ballStatus.x, ballStatus.y];
    };

    this.robotClicked = function(robot)
    {
        div = $('.robot-'+robot.id);

        if (!div.find('.infos').is(':visible')) {
            div.find('.expand').click();
        }
    };

    this.robotsPanel = function(init)
    {
        if (init) {
            var template = $('.robots').html();
            var html = '';

            for (var id=0; id<8; id++) {
                var robotHtml = template;
                robotHtml = robotHtml.replace(/{{id}}/g, id);
                html += robotHtml;
            }

            $('.robots').html(html);

            $('.robots .expand').click(function() {
                var id = $(this).attr('rel');
                var visible = $('.robot-'+id+' .infos').is(':visible');
                $('.robots .infos').hide();
                $('.robots .icon-collapse').hide();
                $('.robots .icon-expand').show();
                $('.robot').removeClass('expanded');

                if (visible) {
                    $('.robot-'+id+' .infos').hide();
                } else {
                    $('.robot-'+id).addClass('expanded');
                    $('.robot-'+id+' .infos').show();
                    $('.robot-'+id+' .icon-collapse').show();
                    $('.robot-'+id+' .icon-expand').hide();
                }
            });

            $('.robots .enable-disable').change(function() {
                var id = parseInt($(this).attr('rel'));
                var robot = robotById(id);

                api.enableRobot(robot.id, !robot.enabled);
                api.robotCommand(robot.id, 0.0, 0.0, 0.0);
                $(this).prop('checked', robot.enabled);
            });

            $('.robots .manual-control').change(function() {
                var id = parseInt($(this).attr('rel'));
                var robot = robotById(id);

                api.manualControl(robot.id, $(this).is(':checked'));
            });

            $('.robots .active').change(function() {
                var id = parseInt($(this).attr('rel'));
                var robot = robotById(id);

                api.activeRobot(robot.id, $(this).is(':checked'));
            });

            var button = function(selector, callback, evt) {
                if (typeof(evt) == 'undefined') {
                    evt = 'click';
                }
                $('.robots '+selector).on(evt, function() {
                    var id = parseInt($(this).attr('rel'));
                    var robot = robotById(id);

                    if (robot) {
                        callback(robot);
                    }
                });
            };

            button('.spin', function(robot) {
                if (robot.enabled) {
                    api.setSpin(robot.id, !robot.spin);
                }
            });

            button('.pid', function(robot) {
                api.tweakPid(robot.id);
            });

            button('.charge', function(robot) {
                if (robot.enabled) {
                    robot.charge = !robot.charge;
                }
                api.robotCharge(robot.id, robot.charge);
            });

            button('.kick100', function(robot) {
                api.kick(robot.id, 1, 1.0);
            }, 'mousedown');
            button('.kick66', function(robot) {
                //api.kick(robot.id, 1, 0.6666);//66.66/100.0);
                api.kick(robot.id, 1, 0.6666);//66.66/100.0);
            }, 'mousedown');
            button('.kick33', function(robot) {
                api.kick(robot.id, 1, 0.3333);
            }, 'mousedown');

            button('.kick-chip100', function(robot) {
                api.kick(robot.id, 2, 1.0);
            }, 'mousedown');
            button('.kick-chip66', function(robot) {
                api.kick(robot.id, 2, 0.6666);
            }, 'mousedown');
            button('.kick-chip33', function(robot) {
                api.kick(robot.id, 2, 0.3333);
            }, 'mousedown');

            button('.joystickize', function(robot) {
                $('.joystick-robot').val(robot.id+'');
                $('.joystick-close').click();
                $('.joystick-open').click();
            });

            button('.kick100, .kick66, .kick33, .kick-chip100, .kick-chip66, .kick-chip33', function(robot) {
                api.kick(robot.id, 0, 0.0);
            }, 'mouseup');

            button('.tare-odom', function(robot) {
                if (robot.enabled) {
                    api.tareOdom(robot.id, false, 0.0, 0.0, 0.0);
                }
            }, 'mouseup');

            button('.tare-odom', function(robot) {
                var ephe = normalizeTheta(robot.orientation);
                if (robot.enabled) {
                    api.tareOdom(robot.id, true, robot.x, robot.y, ephe);
                }
            }, 'mousedown');

            button('.left', function(robot) {
                api.robotCommand(robot.id, 0.0, 0.2, 0.0);
            }, 'mousedown');

            button('.right', function(robot) {
                api.robotCommand(robot.id, 0.0, -0.2, 0.0);
            }, 'mousedown');

            button('.rear', function(robot) {
                api.robotCommand(robot.id, -0.2, 0.0, 0.0);
            }, 'mousedown');

            button('.front', function(robot) {
                api.robotCommand(robot.id, 0.2, 0.0, 0.0);
            }, 'mousedown');

            button('.rotate-right', function(robot) {
                api.robotCommand(robot.id, 0.0, 0.0, -2);
            }, 'mousedown');

            button('.rotate-left', function(robot) {
                api.robotCommand(robot.id, 0.0, 0.0, 2);
            }, 'mousedown');

            button('.command', function(robot) {
                api.robotCommand(robot.id, 0.0, 0.0, 0.0);
            }, 'mouseup');

            if (simulation) {
                $('.robots .real').hide();
            }

            $('.scan').click(function() {
                api.scan();
            });
        }

        $('.robots-warning').hide();

        var robotWarning = function(div, text)
        {
            div.find('.robot-warning').show();
            div.find('.robot-warning-text').show();
            div.find('.robot-warning-text').text(text);
            $('.robots-warning').show();
        };

        for (var k in robots[ourColor]) {
            var robot = robots[ourColor][k];

            $('.robot-'+k+' .enable-disable').prop('checked', robot.enabled);
            $('.robot-'+k+' .manual-control').prop('checked', robot.manual);
            $('.robot-'+k+' .active').prop('checked', robot.active);

            $('.robot-'+k+' .enable-disable').prop('disabled', !robot.manual);
            $('.robot-'+k+' .active').prop('disabled', !robot.manual);

            if (robot.manual) {
                $('.robot-'+k+' .manual-control-zone').show();
            } else {
                $('.robot-'+k+' .manual-control-zone').hide();
            }

            var div = $('.robot-'+k);
            div.find('.robot-warning-text').hide();
            div.find('.robot-warning').hide();

            if (robot.present) {
                div.find('.vision-status').addClass('ok');
                div.find('.pos-x').text(robot.x.toFixed(3));
                div.find('.pos-y').text(robot.y.toFixed(3));
                div.find('.pos-orientation').text((normalizeTheta(robot.orientation)*180/Math.PI).toFixed(3));

                div.find('.x_odom').text(robot.x_odom.toFixed(3));
                div.find('.y_odom').text(robot.y_odom.toFixed(3));
                //div.find('.t_odom').text((normalizeTheta(robot.t_odom)).toFixed(3));
                div.find('.t_odom').text((normalizeTheta((robot.t_odom/1000))*(180/Math.PI)).toFixed(3));

            } else {
                div.find('.vision-status').removeClass('ok');
            }

            if (!simulation) {
                if (robot.com) {
                    if (!robot.driversOk) {
                        robotWarning(div, "Drivers errors");
                    }

                    var voltage_min = 3.6*6;
                    var voltage_max = 4.2*6;
                    var charge = (robot.voltage-voltage_min)/(voltage_max-voltage_min);
                    div.find('.x_odom').text(robot.x_odom.toFixed(3));
                    div.find('.y_odom').text(robot.y_odom.toFixed(3));
                    //div.find('.t_odom').text((normalizeTheta(robot.t_odom)*180/Math.PI).toFixed(3)); 
                    div.find('.t_odom').text((normalizeTheta((robot.t_odom/1000))*(180/Math.PI)).toFixed(3));

                    if (charge < 0) charge = 0;
                    if (charge > 1) charge = 1;

                    div.find('.voltage-progress').text(robot.voltage+' V');
                    div.find('.voltage-progress').css('width', Math.round(charge*100)+'%');

                    if (charge < 0.15) {
                        robotWarning(div, "Low voltage "+robot.voltage+"V");
                    }

                    var cap_max = 180;
                    capCharge = robot.capVoltage/cap_max;
                    if (capCharge > 1) capCharge = 1;

                    div.find('.capacitors-progress').text(robot.capVoltage+' V');
                    div.find('.capacitors-progress').css('width', Math.round(capCharge*100)+'%');

                    div.find('.com-status').addClass('ok');
                } else {
                    if (robot.enabled) {
                        robotWarning(div, "No com");
                    }
                    div.find('.com-status').removeClass('ok');
                }

                var chargeDiv = div.find('.charge');
                if (robot.charge && chargeDiv.hasClass('btn-success')) {
                    chargeDiv.removeClass('btn-success');
                    chargeDiv.addClass('btn-danger');
                    chargeDiv.text('Stop charge');
                }
                if (!robot.charge && chargeDiv.hasClass('btn-danger')) {
                    chargeDiv.removeClass('btn-danger');
                    chargeDiv.addClass('btn-success');
                    chargeDiv.text('Charge');
                }
            }

            var spin = div.find('.spin');
            if (robot.spin) {
                if (spin.hasClass('btn-success')) {
                    spin.removeClass('btn-success');
                    spin.addClass('btn-danger');
                    spin.text('Stop spinning');
                }
            } else {
                if (spin.hasClass('btn-danger')) {
                    spin.addClass('btn-success');
                    spin.removeClass('btn-danger');
                    spin.text('Spin');
                }
            }
        }
    };

    this.optionsPanel = function(init)
    {
        var viewer = this.viewer;

        if (init) {
            $('.reverse-view').change(function() {
                viewer.reversed = $(this).is(':checked');
            });
            $('.greenred-mode').change(function() {
                viewer.greenred = $(this).is(':checked');
            });

            $('.reset-view').click(function() {
                viewer.resetRatio();
            });
            $('.drag-lock').click(function() {
                viewer.dragLock = !viewer.dragLock;

                if (viewer.dragLock) {
                    $(this).text('Unlock the drag');
                    $(this).addClass('btn-success');
                    $(this).removeClass('btn-danger');
                } else {
                    $(this).text('Lock the drag');
                    $(this).addClass('btn-danger');
                    $(this).removeClass('btn-success');
                }
            });

            var joysticks = JSON.parse(api.availableJoysticks());
            var html = '';
            for (var k in joysticks) {
                html += '<option value="'+joysticks[k]+'">'+joysticks[k]+'</option>';
            }
            $('.joysticks').html(html);

            $('.joystick-open').click(function() {
                api.openJoystick(parseInt($('.joystick-robot').val()), $('.joysticks').val());
            });

            $('.joystick-close').click(function() {
                api.stopJoystick();
            });
        }

        $('.we-are-color').text('We are: '+ourColor);
    };

    this.viewer = viewer;

    // Making panel collapsable and expandable
    $('.panel-content.not-shown').hide();

    $('.panel-block').each(function() {
        var content = $(this).find('.panel-content');
        var title = $(this).find('.panel-title');

        if (content.is(':visible')) {
            title.find('h4').prepend('<img class="expand-collapse" src="collapse.png"/>');
        } else {
            title.find('h4').prepend('<img class="expand-collapse" src="expand.png"/>');
        }

        title.click(function() {
            if (content.is(':visible')) {
                content.hide();
                title.find('h4 .expand-collapse').attr('src', 'expand.png');
            } else {
                content.show();
                title.find('h4 .expand-collapse').attr('src', 'collapse.png');
            }
        });
    });

    this.refereePanel = function()
    {
        $('.referee-warning').show();
    };

    this.visionPanel = function()
    {
        var vision = JSON.parse(api.visionStatus());

        // Updating vison panel
        if (vision.hasData) {
            $('.vision-warning').hide();
            $('.vision-infos').text(vision.packets+' packets received');
        } else {
            $('.vision-warning').show();
        }
    };

    this.refereePanel = function()
    {
        var referee = JSON.parse(api.refereeStatus());

        // Updating referee panel
        if (referee.hasData) {
            $('.referee-warning').hide();
            var html = '';
            html += '<b>Stage</b>: '+referee.stage+'<br/>';
            html += '<b>Time left</b>: '+(referee.time_left/1000000.0).toFixed(2)+'s<br/>';
            html += '<b>Command</b>: '+referee.command+'<br/>';
            html += '<hr/>';
            html += referee.packets+' packets received';
            ourColor = referee.our_color;
            yellowName = referee.yellow_name;
            blueName = referee.blue_name;

            bluePositive = referee.blue_positive;
            $('.referee-infos').html(html);
        } else {
            $('.referee-warning').show();
        }
    };

    this.communicationPanel = function(init)
    {
        if (init) {
            $('.com-warning').show();
        }
    };

    this.strategiesPanel = function(init)
    {
        if (init) {
            // Updating managers
            var managers = JSON.parse(api.getManagers());
            var options = '';

            for (var k in managers) {
                var manager = managers[k];
                options += '<option value="'+manager+'">'+manager+'</option>';
            }

            // Updating robots
            var template = $('.robot-strategies').html();
            var html = '';
            for (var id=0; id<8; id++) {
                html += template.replace(/{{id}}/g, id);
            }
            $('.robot-strategies').html(html);

            // Updating strategies
            var strategiesAvailable = JSON.parse(api.getStrategies());
            var strategiesOptions = '<option value="">-</otpion>';

            for (var n in strategiesAvailable) {
                var strategy = strategiesAvailable[n];
                strategiesOptions += '<option value="'+strategy+'">'+strategy+'</option>';
            }

            $('#managers').html(options);
            $('.strategies-selector').html(strategiesOptions);
            var self = this;

            $('.clear-strategy').click(function() {
                var id = $(this).attr('rel');
                $('.strategy-'+id).val('');
                $('apply-'+id).click();
            });

            $('.apply-strategy').click(function() {
                var id = $(this).attr('rel');
                var strategy = $('.strategy-'+id).val();
                api.applyStrategy(id, strategy);
            });

            $('.control-play').click(function() {
                api.setManager($('#managers').val());
                api.managerPlay();
            });
            $('.control-stop').click(function() {
                api.managerStop();
                $('#managers').val('Manual');
                $('.strategies-selector').val('');
            });
        }
    };

    this.divider = 10;

    this.update = function(init)
    {
        this.updateApi();

        // Handling robots management panel
        this.robotsPanel(init);

        this.divider++;
        if (this.divider >= 10) {
            // Viewer options panel
            this.optionsPanel(init);

            // Referee panel
            this.refereePanel(init);

            // Vision panel
            this.visionPanel(init);

            // Communication panel
            this.communicationPanel(init);

            // Strategies panel
            this.strategiesPanel(init);

            this.divider = 0;
        }
    };

    this.update(true);
}

$(document).ready(function() {
    // What is our color ?
    ourColor = api.isYellow() ? 'yellow' : 'blue';

    // Are we in simulation mode
    simulation = api.isSimulation();

    if (simulation) {
        $('.simulation-mode').show();
        $('.communication').hide();
    }

    $('.emergency-button a').click(function() {
        api.emergencyStop();

        for (var k in robots[ourColor]) {
            robots[ourColor][k].spin = false;
        }

        $('.robots .enable-disable').prop('checked', false);
    });

    // Instantiating the viewer
    var viewer = new Viewer();

    // Panels manager
    var manager = new Manager(viewer);

    viewer.manager = manager;

    setInterval(function() {
        manager.update();
        viewer.update();
    }, 20);
});
