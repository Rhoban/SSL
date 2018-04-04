// Field geometry
var field = {
    length: 9.0,
    width: 6.0,
    borders: 0.5,
    centerCircleRadius: 1,
    goalQuarterCircleRadius: 1,
    goalLinearWidth: 0.5,
    goalWidth: 1,
    goalDepth: 0.18,
    goalThickness: 0.023,
    linesThickness: 0.02
};

// Robots
var robots = [
    {
        id: 1,
        x: -2,
        y: 2,
        orientation: 0,
        color: 'blue',
        present: true,
        enabled: true
    },
    {
        id: 2,
        x: -2,
        y: -2,
        orientation: 0,
        color: 'blue',
        present: true,
        enabled: true
    },
    {
        id: 2,
        x: 2,
        y: 2,
        orientation: Math.PI,
        color: 'yellow',
        present: false,
        enabled: true
    },
    {
        id: 3,
        x: 2,
        y: -1,
        orientation: Math.PI/1.5,
        color: 'yellow',
        present: true,
        enabled: true
    }
];

// Our team color
var ourColor = 'yellow';

// Ball
var ball = [0, 0];

function robotById(id, color)
{
    if (typeof(color) == 'undefined') {
        color = ourColor;
    }

    for (var k in robots) {
        var robot = robots[k];

        if (robot.id == id && robot.color == color) {
            return robot;
        }
    }

    return null;
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

        ctx.fillStyle = '#aaa';
        ctx.font = '12pt sans';
        if (this.mousePos) {
            var pos = this.mousePosMeters();

            ctx.fillText("X: "+pos[0].toFixed(2)+"m, Y: "+pos[1].toFixed(2)+"m", 10, 20);

            // Moving the view offset
            if (this.dragging) {
                var dx = (this.mousePos[0] - this.dragBegin[0])/this.ratio;
                var dy = -(this.mousePos[1]- this.dragBegin[1])/this.ratio;
                this.viewOffset = [this.startOffset[0] + dx, this.startOffset[1] + dy];
            }
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

        ctx.restore();

        // Drawing text messages
        ctx.save();
        ctx.translate(this.width/2, this.height/2);
        ctx.translate(this.ratio*this.viewOffset[0], -this.ratio*this.viewOffset[1]);

        for (var k in this.texts) {
            var text = this.texts[k];
            ctx.beginPath();
            ctx.font = (this.ratio/10)+'pt sans';
            if ('color' in text) {
                ctx.fillStyle = text.color;
            }
            ctx.fillText(text.text, text.x*this.ratio, text.y*this.ratio);
            ctx.fill();
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

        function drawGoals(ctx, color) {
            // Goals zone
            ctx.strokeStyle = '#aaa';
            ctx.lineWidth = field.linesThickness;
            ctx.beginPath();
            ctx.arc(-field.length/2, -field.goalLinearWidth/2, field.goalQuarterCircleRadius, -Math.PI/2, 0);
            ctx.lineTo(-(field.length/2)+field.goalQuarterCircleRadius, field.goalLinearWidth/2);
            ctx.arc(-field.length/2, field.goalLinearWidth/2, field.goalQuarterCircleRadius, 0, Math.PI/2);
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
        drawGoals(ctx, this.reversed ? this.grColor('blue') : this.grColor('yellow'));
        ctx.save();
        ctx.scale(-1, 1);
        drawGoals(ctx, this.reversed ? this.grColor('yellow') : this.grColor('blue'));
        ctx.restore();
    };

    this.drawRobots = function()
    {
        var ctx = this.ctx;
        var over = this.overRobot();

        for (var k in robots) {
            var robot = robots[k];
            this.drawRobot(robot, over == robot);
        }
    };

    this.drawRobot = function(robot, extra)
    {
        var ctx = this.ctx;
        var front = 0.75;

        if (!('ghost' in robot) && this.rotatingRobot && this.rotatingRobot[0] == robot.id &&
            this.rotatingRobot[1] == robot.color) {
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
        ctx.fillStyle = this.grColor(robot.color);
        ctx.arc(robot.x, robot.y, 0.1, robot.orientation+front, robot.orientation+Math.PI*2-front);
        if (!robot.present) {
            ctx.stroke();
        }
        ctx.fill();

        ctx.restore();
        var color = robot.present ? '#333' : 'white';

        this.addText(''+robot.id,this.sign*robot.x-0.05, -this.sign*robot.y+0.055, {color: color});

        if (typeof(extra) != 'undefined' && extra) {
            ctx.fillStyle = '#aaa';
            this.addText(robot.color+' #'+robot.id, this.sign*robot.x+0.15, -this.sign*robot.y-0.15, {color: '#aaa'});
            this.addText('x:'+this.sign*robot.x.toFixed(2)+'m', this.sign*robot.x+0.15, -this.sign*robot.y, {color: '#aaa'});
            this.addText('y:'+this.sign*robot.y.toFixed(2)+'m', this.sign*robot.x+0.15, -this.sign*robot.y+0.15, {color: '#aaa'});
            this.addText('t:'+(180*robot.orientation/Math.PI).toFixed(1)+'°', this.sign*robot.x+0.1, 5, -this.sign*robot.y+0.3, {color: '#aaa'});
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
            for (var k in robots) {
                var robot = robots[k];
                var dist = Math.sqrt(Math.pow(robot.x-xy[0], 2) + Math.pow(robot.y-xy[1], 2));
                if (dist < 0.1) {
                    return robot;
                }
            }
        }

        return null;
    };

    // Mouse is down
    this.mouseDown = function(evt)
    {
        console.log(evt);

        var over = false;
        if (this.mousePos) {
            this.dragBegin = this.mousePos;

            if (evt.originalEvent.which == 1) {
                if (this.overBall()) {
                    this.draggingBall = true;
                } else {
                    over = this.overRobot();
                    if (over) {
                        this.draggingRobot = [over.id, over.color];
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
                    this.rotatingRobot = [over.id, over.color];
                }
            }
        }
    };

    // Stopping the dragging
    this.mouseUp = function(evt)
    {
        var robot = null;
        var pos = null;

        if (this.draggingRobot && this.mousePos) {
            robot = robotById(this.draggingRobot[0], this.draggingRobot[1]);
            if (robot) {
                pos = this.mousePosMeters();

                // XXX: Communicate with the API
                robot.x = pos[0];
                robot.y = pos[1];
            }
        }

        if (this.rotatingRobot && this.mousePos) {
            robot = robotById(this.rotatingRobot[0], this.rotatingRobot[1]);
            if (robot) {
                var angle = this.mouseAngle(robot);

                // XXX: Communicate with the API
                robot.orientation = angle;
            }
        }

        if (this.draggingBall && this.mousePos) {
            pos = this.mousePosMeters();

            // XXX: Communicate with the API
            ball = pos;
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
        console.log(evt);
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
    setInterval(function() { self.update(); }, 20);

    $('#field').mousemove(function(evt) { self.mouseMove(evt); });
    $('#field').mousedown(function(evt) { self.mouseDown(evt); });
    $('#field').mouseup(function(evt) { self.mouseUp(evt); });
    $('#field').mousewheel(function(evt) { self.mouseWheel(evt); });

    document.addEventListener('keypress', function(evt) { self.keyPress(evt); });
}

function Manager(viewer)
{
    this.robotsPanel = function()
    {
        var template = $('.robots').html();
        var html = '';

        for (var id=1; id<=8; id++) {
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

        $('.robots .enable-disable').click(function() {
            var id = parseInt($(this).attr('rel'));
            var robot = robotById(id);

            if (robot) {
                robot.enabled = !robot.enabled;

                if (robot.enabled) {
                    $(this).removeClass('btn-success');
                    $(this).addClass('btn-danger');
                    $(this).text('Disable');
                } else {
                    $(this).addClass('btn-success');
                    $(this).removeClass('btn-danger');
                    $(this).text('Enable');
                }
            }
        });
    };

    this.optionsPanel = function()
    {
        var viewer = this.viewer;

        $('.reverse-view').change(function() {
            viewer.reversed = $(this).is(':checked');
        });
        $('.greenred-mode').change(function() {
            viewer.greenred = $(this).is(':checked');
        });
        $('.we-are-color').click(function() {
            if (ourColor == 'yellow') {
                ourColor = 'blue';
            } else {
                ourColor = 'yellow';
            }
            $(this).text('We are: '+ourColor);
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

    // Handling robots management panel
    this.robotsPanel();

    // Viewer options panel
    this.optionsPanel();
}

$(document).ready(function() {
    // Instantiating the viewer
    var viewer = new Viewer();

    // Panels manager
    var manager = new Manager(viewer);
});
