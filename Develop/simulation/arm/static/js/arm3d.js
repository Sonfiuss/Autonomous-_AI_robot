/**
 * 6DOF DIY Robot Arm 3D Visualization
 * Realistic mini robot arm kit style with RC servos
 * Uses Three.js for 3D rendering
 */

class Arm3DVisualization {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.armGroup = null;
        this.jointMeshes = [];
        this.linkMeshes = [];
        this.gridHelper = null;
        this.axesHelper = null;
        
        // Raycaster for click detection
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();
        this.groundPlane = null;
        
        // Objects placed on the grid
        this.placedObjects = [];
        this.selectedObject = null;
        
        // Arm reach parameters
        this.maxReach = 380;
        this.minReach = 30;
        
        // Materials - DIY kit style
        this.materials = {
            // Black matte plastic (slightly rough)
            blackPlastic: new THREE.MeshStandardMaterial({
                color: 0x1a1a1a,
                roughness: 0.85,
                metalness: 0.05
            }),
            // Dark gray plastic
            darkGrayPlastic: new THREE.MeshStandardMaterial({
                color: 0x2d2d2d,
                roughness: 0.8,
                metalness: 0.1
            }),
            // Servo blue (like SG90/MG996R)
            servoBlue: new THREE.MeshStandardMaterial({
                color: 0x1e3a5f,
                roughness: 0.6,
                metalness: 0.2
            }),
            // Brushed aluminum
            aluminum: new THREE.MeshStandardMaterial({
                color: 0xc0c0c0,
                roughness: 0.4,
                metalness: 0.8
            }),
            // Steel screws
            steel: new THREE.MeshStandardMaterial({
                color: 0x707070,
                roughness: 0.3,
                metalness: 0.9
            }),
            // Servo horn (white/cream plastic)
            horn: new THREE.MeshStandardMaterial({
                color: 0xf5f5dc,
                roughness: 0.7,
                metalness: 0.0
            }),
            // Brown cable
            cableBrown: new THREE.MeshStandardMaterial({
                color: 0x4a3728,
                roughness: 0.9,
                metalness: 0.0
            }),
            // Red cable
            cableRed: new THREE.MeshStandardMaterial({
                color: 0xb91c1c,
                roughness: 0.9,
                metalness: 0.0
            }),
            // Orange cable
            cableOrange: new THREE.MeshStandardMaterial({
                color: 0xea580c,
                roughness: 0.9,
                metalness: 0.0
            }),
            // Object colors
            object: new THREE.MeshStandardMaterial({
                color: 0xf59e0b,
                roughness: 0.5,
                metalness: 0.3
            })
        };
        
        // Legacy colors for compatibility
        this.colors = {
            object: 0xf59e0b,
            objectSelected: 0x22c55e,
            objectOutOfRange: 0xef4444
        };
        
        this.init();
        this.createScene();
        this.setupClickHandler();
        this.animate();
    }
    
    init() {
        // Scene - clean white studio background
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0xfafafa);
        
        // Camera - 3/4 view angle
        const aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera = new THREE.PerspectiveCamera(45, aspect, 0.1, 5000);
        this.camera.position.set(350, 280, 380);
        this.camera.lookAt(0, 120, 0);
        
        // Renderer with high quality settings
        this.renderer = new THREE.WebGLRenderer({ 
            antialias: true,
            alpha: true
        });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.0;
        this.renderer.outputColorSpace = THREE.SRGBColorSpace;
        this.container.appendChild(this.renderer.domElement);
        
        // Orbit Controls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.target.set(0, 120, 0);
        this.controls.update();
        
        // Studio lighting setup
        // Key light (main light)
        const keyLight = new THREE.DirectionalLight(0xffffff, 1.2);
        keyLight.position.set(250, 400, 300);
        keyLight.castShadow = true;
        keyLight.shadow.mapSize.width = 2048;
        keyLight.shadow.mapSize.height = 2048;
        keyLight.shadow.camera.near = 50;
        keyLight.shadow.camera.far = 1200;
        keyLight.shadow.camera.left = -400;
        keyLight.shadow.camera.right = 400;
        keyLight.shadow.camera.top = 400;
        keyLight.shadow.camera.bottom = -400;
        keyLight.shadow.bias = -0.0001;
        this.scene.add(keyLight);
        
        // Fill light (softer, from opposite side)
        const fillLight = new THREE.DirectionalLight(0xffffff, 0.5);
        fillLight.position.set(-200, 250, -150);
        this.scene.add(fillLight);
        
        // Rim/back light
        const rimLight = new THREE.DirectionalLight(0xffffff, 0.3);
        rimLight.position.set(0, 300, -350);
        this.scene.add(rimLight);
        
        // Ambient light for soft fill
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
        this.scene.add(ambientLight);
        
        // Hemisphere light for natural sky/ground bounce
        const hemiLight = new THREE.HemisphereLight(0xffffff, 0xe0e0e0, 0.3);
        this.scene.add(hemiLight);
        
        // Handle window resize
        window.addEventListener('resize', () => this.onWindowResize());
    }
    
    createScene() {
        // Create subtle ground plane
        this.createGround();
        
        // Create arm group
        this.armGroup = new THREE.Group();
        this.scene.add(this.armGroup);
        
        // Create the complete DIY robot arm
        this.createDIYRobotArm();
        
        // Initialize arm with default positions
        this.updateArm([
            [0, 0, 0],
            [0, 50, 0],
            [0, 50, 0],
            [0, 150, 0],
            [0, 250, 0],
            [0, 280, 0],
            [0, 300, 0]
        ]);
    }
    
    createGround() {
        // Subtle reflective ground
        const groundGeometry = new THREE.PlaneGeometry(1000, 1000);
        const groundMaterial = new THREE.MeshStandardMaterial({
            color: 0xf5f5f5,
            roughness: 0.1,
            metalness: 0.0
        });
        this.groundPlane = new THREE.Mesh(groundGeometry, groundMaterial);
        this.groundPlane.rotation.x = -Math.PI / 2;
        this.groundPlane.position.y = -1;
        this.groundPlane.receiveShadow = true;
        this.groundPlane.name = 'ground';
        this.scene.add(this.groundPlane);
        
        // Subtle grid for scale reference
        const gridSize = 600;
        const gridDivisions = 15;
        this.gridHelper = new THREE.GridHelper(gridSize, gridDivisions, 0xdddddd, 0xeeeeee);
        this.gridHelper.position.y = 0;
        this.scene.add(this.gridHelper);
    }
    
    // =============================================
    // DIY ROBOT ARM COMPONENTS
    // =============================================
    
    createDIYRobotArm() {
        // Scale: ~280mm total height (scale factor for Three.js units)
        // This creates the static base structure
        this.createBaseWithServo();
    }
    
    // Square base plate with mounting tabs and base servo
    createBaseWithServo() {
        const baseGroup = new THREE.Group();
        
        // Main base plate - square black plastic
        const plateWidth = 80;
        const plateDepth = 80;
        const plateHeight = 8;
        
        const plateGeometry = new THREE.BoxGeometry(plateWidth, plateHeight, plateDepth);
        const plate = new THREE.Mesh(plateGeometry, this.materials.blackPlastic);
        plate.position.y = plateHeight / 2;
        plate.castShadow = true;
        plate.receiveShadow = true;
        baseGroup.add(plate);
        
        // 4 mounting tabs with holes
        const tabPositions = [
            { x: -plateWidth/2 - 8, z: -plateDepth/2 - 8 },
            { x: plateWidth/2 + 8, z: -plateDepth/2 - 8 },
            { x: -plateWidth/2 - 8, z: plateDepth/2 + 8 },
            { x: plateWidth/2 + 8, z: plateDepth/2 + 8 }
        ];
        
        tabPositions.forEach(pos => {
            const tab = this.createMountingTab();
            tab.position.set(pos.x, plateHeight/2, pos.z);
            baseGroup.add(tab);
        });
        
        // Base servo (for yaw rotation) - larger servo like MG996R
        const baseServo = this.createServoMG996R();
        baseServo.position.set(0, plateHeight + 12, 0);
        baseGroup.add(baseServo);
        
        // Servo horn on top
        const baseHorn = this.createServoHornRound();
        baseHorn.position.set(0, plateHeight + 28, 0);
        baseServo.add(baseHorn);
        
        // Turntable plate on top of servo
        const turntable = this.createTurntablePlate();
        turntable.position.set(0, plateHeight + 32, 0);
        baseGroup.add(turntable);
        
        this.armGroup.add(baseGroup);
    }
    
    // Mounting tab with screw hole
    createMountingTab() {
        const group = new THREE.Group();
        
        // Tab body
        const tabGeometry = new THREE.CylinderGeometry(8, 8, 8, 16);
        const tab = new THREE.Mesh(tabGeometry, this.materials.blackPlastic);
        tab.rotation.x = Math.PI / 2;
        tab.castShadow = true;
        group.add(tab);
        
        // Screw hole (dark indent)
        const holeGeometry = new THREE.CylinderGeometry(3, 3, 9, 16);
        const holeMaterial = new THREE.MeshStandardMaterial({ color: 0x0a0a0a });
        const hole = new THREE.Mesh(holeGeometry, holeMaterial);
        group.add(hole);
        
        return group;
    }
    
    // Standard RC Servo (MG996R style) - 40x20x36mm
    createServoMG996R() {
        const group = new THREE.Group();
        
        // Main servo body - blue
        const bodyWidth = 40;
        const bodyHeight = 20;
        const bodyDepth = 36;
        
        const bodyGeometry = new THREE.BoxGeometry(bodyWidth, bodyHeight, bodyDepth);
        const body = new THREE.Mesh(bodyGeometry, this.materials.servoBlue);
        body.castShadow = true;
        group.add(body);
        
        // Mounting ears
        const earWidth = 54;
        const earHeight = 3;
        const earDepth = 8;
        const earGeometry = new THREE.BoxGeometry(earWidth, earHeight, earDepth);
        const ear = new THREE.Mesh(earGeometry, this.materials.servoBlue);
        ear.position.y = bodyHeight/2 - earHeight/2;
        ear.position.z = -bodyDepth/2 + earDepth/2 + 2;
        group.add(ear);
        
        // Ear mounting holes
        const earHoleGeometry = new THREE.CylinderGeometry(2, 2, 4, 8);
        [-20, 20].forEach(x => {
            const hole = new THREE.Mesh(earHoleGeometry, this.materials.steel);
            hole.position.set(x, bodyHeight/2 - earHeight/2, ear.position.z);
            group.add(hole);
        });
        
        // Output shaft housing (cylinder on top)
        const shaftHousingGeometry = new THREE.CylinderGeometry(8, 8, 5, 16);
        const shaftHousing = new THREE.Mesh(shaftHousingGeometry, this.materials.darkGrayPlastic);
        shaftHousing.position.set(0, bodyHeight/2 + 2.5, -4);
        group.add(shaftHousing);
        
        // Output shaft
        const shaftGeometry = new THREE.CylinderGeometry(3, 3, 4, 12);
        const shaft = new THREE.Mesh(shaftGeometry, this.materials.steel);
        shaft.position.set(0, bodyHeight/2 + 6, -4);
        group.add(shaft);
        
        // Screws on mounting ears
        this.addHexScrews(group, [
            { x: -22, y: bodyHeight/2, z: ear.position.z },
            { x: 22, y: bodyHeight/2, z: ear.position.z }
        ]);
        
        // Wire connector
        const connectorGeometry = new THREE.BoxGeometry(10, 5, 3);
        const connector = new THREE.Mesh(connectorGeometry, this.materials.darkGrayPlastic);
        connector.position.set(0, 0, bodyDepth/2);
        group.add(connector);
        
        // Servo wires (3-wire: brown, red, orange)
        const cableBundle = this.createServoCable(30);
        cableBundle.position.set(0, 0, bodyDepth/2 + 3);
        cableBundle.rotation.x = Math.PI / 2;
        group.add(cableBundle);
        
        return group;
    }
    
    // Micro servo (SG90 style) - 23x12x22mm
    createServoSG90() {
        const group = new THREE.Group();
        
        // Main servo body
        const bodyWidth = 23;
        const bodyHeight = 12;
        const bodyDepth = 22;
        
        const bodyGeometry = new THREE.BoxGeometry(bodyWidth, bodyHeight, bodyDepth);
        const body = new THREE.Mesh(bodyGeometry, this.materials.servoBlue);
        body.castShadow = true;
        group.add(body);
        
        // Mounting ears
        const earWidth = 32;
        const earHeight = 2.5;
        const earDepth = 5;
        const earGeometry = new THREE.BoxGeometry(earWidth, earHeight, earDepth);
        const ear = new THREE.Mesh(earGeometry, this.materials.servoBlue);
        ear.position.y = bodyHeight/2 - earHeight/2;
        group.add(ear);
        
        // Output shaft housing
        const shaftHousingGeometry = new THREE.CylinderGeometry(5, 5, 4, 12);
        const shaftHousing = new THREE.Mesh(shaftHousingGeometry, this.materials.darkGrayPlastic);
        shaftHousing.position.set(0, bodyHeight/2 + 2, -3);
        group.add(shaftHousing);
        
        // Output shaft
        const shaftGeometry = new THREE.CylinderGeometry(2.5, 2.5, 3, 10);
        const shaft = new THREE.Mesh(shaftGeometry, this.materials.steel);
        shaft.position.set(0, bodyHeight/2 + 4.5, -3);
        group.add(shaft);
        
        // Wire connector
        const connectorGeometry = new THREE.BoxGeometry(8, 4, 2);
        const connector = new THREE.Mesh(connectorGeometry, this.materials.darkGrayPlastic);
        connector.position.set(0, -2, bodyDepth/2);
        group.add(connector);
        
        return group;
    }
    
    // Round servo horn (multi-arm style)
    createServoHornRound() {
        const group = new THREE.Group();
        
        // Center hub
        const hubGeometry = new THREE.CylinderGeometry(6, 6, 3, 16);
        const hub = new THREE.Mesh(hubGeometry, this.materials.horn);
        hub.castShadow = true;
        group.add(hub);
        
        // Arms (4-way)
        for (let i = 0; i < 4; i++) {
            const armGeometry = new THREE.BoxGeometry(4, 2, 14);
            const arm = new THREE.Mesh(armGeometry, this.materials.horn);
            arm.rotation.y = (i * Math.PI) / 2;
            arm.position.y = 1;
            group.add(arm);
            
            // Hole at end of arm
            const holeGeometry = new THREE.CylinderGeometry(1, 1, 3, 8);
            const hole = new THREE.Mesh(holeGeometry, this.materials.steel);
            const angle = (i * Math.PI) / 2;
            hole.position.set(Math.sin(angle) * 12, 1, Math.cos(angle) * 12);
            group.add(hole);
        }
        
        // Center screw
        const screwGeometry = new THREE.CylinderGeometry(2, 2, 4, 6);
        const screw = new THREE.Mesh(screwGeometry, this.materials.steel);
        screw.position.y = 3;
        group.add(screw);
        
        return group;
    }
    
    // Single arm servo horn
    createServoHornSingle() {
        const group = new THREE.Group();
        
        // Hub
        const hubGeometry = new THREE.CylinderGeometry(4, 4, 3, 12);
        const hub = new THREE.Mesh(hubGeometry, this.materials.horn);
        group.add(hub);
        
        // Single arm
        const armGeometry = new THREE.BoxGeometry(4, 2, 20);
        const arm = new THREE.Mesh(armGeometry, this.materials.horn);
        arm.position.set(0, 1, 8);
        group.add(arm);
        
        // Holes along arm
        [5, 10, 15].forEach(z => {
            const holeGeometry = new THREE.CylinderGeometry(1, 1, 3, 8);
            const hole = new THREE.Mesh(holeGeometry, this.materials.steel);
            hole.position.set(0, 1, z);
            group.add(hole);
        });
        
        return group;
    }
    
    // Turntable plate that sits on base servo
    createTurntablePlate() {
        const group = new THREE.Group();
        
        const plateGeometry = new THREE.CylinderGeometry(35, 35, 5, 24);
        const plate = new THREE.Mesh(plateGeometry, this.materials.blackPlastic);
        plate.castShadow = true;
        group.add(plate);
        
        // Mounting holes around edge
        for (let i = 0; i < 6; i++) {
            const angle = (i * Math.PI * 2) / 6;
            const holeGeometry = new THREE.CylinderGeometry(2, 2, 6, 8);
            const hole = new THREE.Mesh(holeGeometry, this.materials.steel);
            hole.position.set(Math.sin(angle) * 28, 0, Math.cos(angle) * 28);
            group.add(hole);
        }
        
        return group;
    }
    
    // U-bracket for shoulder/elbow joints
    createUBracket(width = 30, height = 50, depth = 25) {
        const group = new THREE.Group();
        
        // Thickness of bracket walls
        const t = 4;
        
        // Left wall
        const leftWallGeometry = new THREE.BoxGeometry(t, height, depth);
        const leftWall = new THREE.Mesh(leftWallGeometry, this.materials.blackPlastic);
        leftWall.position.set(-width/2 + t/2, 0, 0);
        leftWall.castShadow = true;
        group.add(leftWall);
        
        // Right wall
        const rightWall = new THREE.Mesh(leftWallGeometry, this.materials.blackPlastic);
        rightWall.position.set(width/2 - t/2, 0, 0);
        rightWall.castShadow = true;
        group.add(rightWall);
        
        // Bottom connecting piece
        const bottomGeometry = new THREE.BoxGeometry(width, t, depth);
        const bottom = new THREE.Mesh(bottomGeometry, this.materials.blackPlastic);
        bottom.position.set(0, -height/2 + t/2, 0);
        bottom.castShadow = true;
        group.add(bottom);
        
        // Bearing holes on sides
        const holeGeometry = new THREE.CylinderGeometry(4, 4, t + 1, 12);
        const holeLeft = new THREE.Mesh(holeGeometry, this.materials.steel);
        holeLeft.rotation.z = Math.PI / 2;
        holeLeft.position.set(-width/2 + t/2, height/4, 0);
        group.add(holeLeft);
        
        const holeRight = holeLeft.clone();
        holeRight.position.x = width/2 - t/2;
        group.add(holeRight);
        
        // Mounting holes on bottom
        this.addMountingHoles(group, [
            { x: -10, y: -height/2 + t/2, z: -8 },
            { x: 10, y: -height/2 + t/2, z: -8 },
            { x: -10, y: -height/2 + t/2, z: 8 },
            { x: 10, y: -height/2 + t/2, z: 8 }
        ]);
        
        return group;
    }
    
    // Arm link (rectangular box with mounting features)
    createArmLink(length, width = 25, height = 20) {
        const group = new THREE.Group();
        
        // Main body
        const bodyGeometry = new THREE.BoxGeometry(width, height, length);
        const body = new THREE.Mesh(bodyGeometry, this.materials.blackPlastic);
        body.castShadow = true;
        group.add(body);
        
        // Lightening holes/slots along length
        const numHoles = Math.floor(length / 25);
        for (let i = 0; i < numHoles; i++) {
            const slotGeometry = new THREE.BoxGeometry(12, height + 2, 8);
            const slotMaterial = new THREE.MeshStandardMaterial({ color: 0x0a0a0a });
            const slot = new THREE.Mesh(slotGeometry, slotMaterial);
            slot.position.z = -length/2 + 20 + i * 25;
            group.add(slot);
        }
        
        // End mounting holes
        this.addMountingHoles(group, [
            { x: -8, y: 0, z: -length/2 + 5 },
            { x: 8, y: 0, z: -length/2 + 5 },
            { x: -8, y: 0, z: length/2 - 5 },
            { x: 8, y: 0, z: length/2 - 5 }
        ]);
        
        return group;
    }
    
    // Gripper assembly with aluminum fingers
    createGripper() {
        const group = new THREE.Group();
        
        // Gripper base/wrist mount
        const baseGeometry = new THREE.BoxGeometry(35, 18, 25);
        const base = new THREE.Mesh(baseGeometry, this.materials.blackPlastic);
        base.castShadow = true;
        group.add(base);
        
        // Micro servo for gripper (mounted on top)
        const gripperServo = this.createServoSG90();
        gripperServo.position.set(0, 15, -5);
        gripperServo.rotation.y = Math.PI;
        group.add(gripperServo);
        
        // Servo horn for gripper actuation
        const horn = this.createServoHornSingle();
        horn.position.set(0, 25, -8);
        horn.rotation.x = Math.PI / 2;
        group.add(horn);
        
        // Left aluminum finger
        const leftFinger = this.createGripperFinger();
        leftFinger.position.set(-15, -5, 15);
        group.add(leftFinger);
        
        // Right aluminum finger (mirrored)
        const rightFinger = this.createGripperFinger();
        rightFinger.position.set(15, -5, 15);
        rightFinger.scale.x = -1;
        group.add(rightFinger);
        
        // Finger linkage bar
        const linkageGeometry = new THREE.BoxGeometry(25, 3, 3);
        const linkage = new THREE.Mesh(linkageGeometry, this.materials.aluminum);
        linkage.position.set(0, 0, 5);
        group.add(linkage);
        
        // Linkage pivots with screws
        this.addHexScrews(group, [
            { x: -10, y: 0, z: 5, size: 2 },
            { x: 10, y: 0, z: 5, size: 2 }
        ]);
        
        return group;
    }
    
    // Single gripper finger (brushed aluminum)
    createGripperFinger() {
        const group = new THREE.Group();
        
        // Finger base plate
        const baseGeometry = new THREE.BoxGeometry(15, 20, 3);
        const base = new THREE.Mesh(baseGeometry, this.materials.aluminum);
        base.castShadow = true;
        group.add(base);
        
        // Finger tip (angled for gripping)
        const tipGeometry = new THREE.BoxGeometry(12, 35, 2);
        const tip = new THREE.Mesh(tipGeometry, this.materials.aluminum);
        tip.position.set(-2, 25, 10);
        tip.rotation.x = -0.3;
        tip.castShadow = true;
        group.add(tip);
        
        // Grip texture (small ridges)
        for (let i = 0; i < 4; i++) {
            const ridgeGeometry = new THREE.BoxGeometry(10, 1, 3);
            const ridge = new THREE.Mesh(ridgeGeometry, this.materials.steel);
            ridge.position.set(-2, 15 + i * 6, 12);
            ridge.rotation.x = -0.3;
            group.add(ridge);
        }
        
        // Mounting hole at base
        const holeGeometry = new THREE.CylinderGeometry(2, 2, 4, 8);
        const hole = new THREE.Mesh(holeGeometry, this.materials.steel);
        hole.rotation.x = Math.PI / 2;
        hole.position.set(0, -5, 0);
        group.add(hole);
        
        // Hex screw
        const screw = this.createHexScrew(2.5);
        screw.position.set(0, -5, 2);
        group.add(screw);
        
        return group;
    }
    
    // Servo cable bundle (3 wires)
    createServoCable(length) {
        const group = new THREE.Group();
        
        const wireRadius = 0.8;
        const spacing = 2;
        const materials = [
            this.materials.cableBrown,
            this.materials.cableRed,
            this.materials.cableOrange
        ];
        
        // Create curved cable path
        const curve = new THREE.CatmullRomCurve3([
            new THREE.Vector3(0, 0, 0),
            new THREE.Vector3(2, length * 0.3, 0),
            new THREE.Vector3(-1, length * 0.6, 0),
            new THREE.Vector3(0, length, 0)
        ]);
        
        const tubeGeometry = new THREE.TubeGeometry(curve, 20, wireRadius, 6, false);
        
        materials.forEach((mat, i) => {
            const wire = new THREE.Mesh(tubeGeometry.clone(), mat);
            wire.position.x = (i - 1) * spacing;
            wire.castShadow = true;
            group.add(wire);
        });
        
        // Cable tie/bundle wrap
        const tieGeometry = new THREE.TorusGeometry(3, 0.5, 4, 12);
        const tieMaterial = new THREE.MeshStandardMaterial({ color: 0x111111 });
        
        [length * 0.3, length * 0.7].forEach(y => {
            const tie = new THREE.Mesh(tieGeometry, tieMaterial);
            tie.position.y = y;
            tie.rotation.x = Math.PI / 2;
            group.add(tie);
        });
        
        return group;
    }
    
    // Hex screw with head detail
    createHexScrew(radius = 2) {
        const group = new THREE.Group();
        
        // Hex head
        const headGeometry = new THREE.CylinderGeometry(radius * 1.5, radius * 1.5, radius, 6);
        const head = new THREE.Mesh(headGeometry, this.materials.steel);
        head.castShadow = true;
        group.add(head);
        
        // Hex socket indent
        const socketGeometry = new THREE.CylinderGeometry(radius * 0.6, radius * 0.6, radius * 0.5, 6);
        const socketMaterial = new THREE.MeshStandardMaterial({ color: 0x404040 });
        const socket = new THREE.Mesh(socketGeometry, socketMaterial);
        socket.position.y = radius * 0.3;
        group.add(socket);
        
        return group;
    }
    
    // Add hex screws to a group at specified positions
    addHexScrews(group, positions) {
        positions.forEach(pos => {
            const screw = this.createHexScrew(pos.size || 2);
            screw.position.set(pos.x, pos.y, pos.z);
            group.add(screw);
        });
    }
    
    // Add mounting holes (dark circular indents)
    addMountingHoles(group, positions) {
        const holeGeometry = new THREE.CylinderGeometry(2, 2, 2, 8);
        const holeMaterial = new THREE.MeshStandardMaterial({ color: 0x0a0a0a });
        
        positions.forEach(pos => {
            const hole = new THREE.Mesh(holeGeometry, holeMaterial);
            hole.position.set(pos.x, pos.y, pos.z);
            group.add(hole);
        });
    }
    
    // =============================================
    // ARM UPDATE AND KINEMATICS
    // =============================================
    
    updateArm(positions) {
        // Clear existing dynamic arm parts
        this.jointMeshes.forEach(mesh => this.armGroup.remove(mesh));
        this.linkMeshes.forEach(mesh => this.armGroup.remove(mesh));
        this.jointMeshes = [];
        this.linkMeshes = [];
        
        if (positions.length < 2) return;
        
        // BASE (position 0) - already created in createBaseWithServo
        
        // SHOULDER JOINT at position 1 - U-bracket with servo
        const shoulderBracket = this.createUBracket(40, 55, 30);
        shoulderBracket.position.set(positions[1][0], positions[1][1] + 30, positions[1][2]);
        this.jointMeshes.push(shoulderBracket);
        this.armGroup.add(shoulderBracket);
        
        // Shoulder servo inside bracket
        const shoulderServo = this.createServoMG996R();
        shoulderServo.position.set(positions[1][0], positions[1][1] + 50, positions[1][2]);
        shoulderServo.rotation.z = Math.PI / 2;
        this.jointMeshes.push(shoulderServo);
        this.armGroup.add(shoulderServo);
        
        // UPPER ARM LINK (position 1 to 3)
        if (positions.length > 3) {
            const upperArmLength = this.getDistance(positions[1], positions[3]);
            const upperArm = this.createArmLink(upperArmLength, 28, 22);
            upperArm.position.set(
                positions[1][0],
                positions[1][1] + 60,
                positions[1][2]
            );
            
            // Rotate to point from shoulder to elbow
            const direction = new THREE.Vector3(
                positions[3][0] - positions[1][0],
                positions[3][1] - positions[1][1],
                positions[3][2] - positions[1][2]
            ).normalize();
            const quaternion = new THREE.Quaternion();
            quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), direction);
            upperArm.setRotationFromQuaternion(quaternion);
            
            this.linkMeshes.push(upperArm);
            this.armGroup.add(upperArm);
            
            // Cable running along upper arm
            const upperCable = this.createServoCable(upperArmLength * 0.8);
            upperCable.position.set(
                positions[1][0] + 15,
                positions[1][1] + 65,
                positions[1][2]
            );
            this.linkMeshes.push(upperCable);
            this.armGroup.add(upperCable);
        }
        
        // ELBOW JOINT at position 3 - servo with bracket
        if (positions.length > 3) {
            const elbowBracket = this.createUBracket(35, 45, 25);
            elbowBracket.position.set(positions[3][0], positions[3][1], positions[3][2]);
            this.jointMeshes.push(elbowBracket);
            this.armGroup.add(elbowBracket);
            
            const elbowServo = this.createServoMG996R();
            elbowServo.position.set(positions[3][0], positions[3][1] + 15, positions[3][2]);
            elbowServo.rotation.z = Math.PI / 2;
            elbowServo.scale.set(0.8, 0.8, 0.8);
            this.jointMeshes.push(elbowServo);
            this.armGroup.add(elbowServo);
        }
        
        // FOREARM LINK (position 3 to 4)
        if (positions.length > 4) {
            const forearmLength = this.getDistance(positions[3], positions[4]);
            const forearm = this.createArmLink(forearmLength, 22, 18);
            forearm.position.set(
                positions[3][0],
                positions[3][1] + 25,
                positions[3][2]
            );
            
            const direction = new THREE.Vector3(
                positions[4][0] - positions[3][0],
                positions[4][1] - positions[3][1],
                positions[4][2] - positions[3][2]
            ).normalize();
            const quaternion = new THREE.Quaternion();
            quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), direction);
            forearm.setRotationFromQuaternion(quaternion);
            
            this.linkMeshes.push(forearm);
            this.armGroup.add(forearm);
            
            // Cable running along forearm
            const forearmCable = this.createServoCable(forearmLength * 0.6);
            forearmCable.position.set(
                positions[3][0] + 12,
                positions[3][1] + 30,
                positions[3][2]
            );
            this.linkMeshes.push(forearmCable);
            this.armGroup.add(forearmCable);
        }
        
        // WRIST JOINT at position 4/5 - small servo
        if (positions.length > 5) {
            const wristServo = this.createServoSG90();
            wristServo.position.set(positions[4][0], positions[4][1], positions[4][2]);
            this.jointMeshes.push(wristServo);
            this.armGroup.add(wristServo);
            
            // Wrist rotation servo
            const wristRotServo = this.createServoSG90();
            wristRotServo.position.set(positions[5][0], positions[5][1], positions[5][2]);
            wristRotServo.rotation.z = Math.PI / 2;
            this.jointMeshes.push(wristRotServo);
            this.armGroup.add(wristRotServo);
        }
        
        // GRIPPER at final position
        if (positions.length > 6) {
            const gripper = this.createGripper();
            gripper.position.set(
                positions[positions.length - 1][0],
                positions[positions.length - 1][1],
                positions[positions.length - 1][2]
            );
            this.jointMeshes.push(gripper);
            this.armGroup.add(gripper);
        }
        
        // Update end effector position display
        const endPos = positions[positions.length - 1];
        const posDisplay = document.getElementById('end-effector-pos');
        if (posDisplay) {
            posDisplay.textContent = `End Effector: X: ${endPos[0].toFixed(1)} Y: ${endPos[1].toFixed(1)} Z: ${endPos[2].toFixed(1)}`;
        }
    }
    
    getDistance(pos1, pos2) {
        return Math.sqrt(
            Math.pow(pos2[0] - pos1[0], 2) +
            Math.pow(pos2[1] - pos1[1], 2) +
            Math.pow(pos2[2] - pos1[2], 2)
        );
    }
    
    // =============================================
    // CAMERA AND VIEW CONTROLS
    // =============================================
    
    setCameraView(view) {
        const distance = 600;
        let position, target;
        
        switch (view) {
            case 'top':
                position = new THREE.Vector3(0, distance, 0);
                target = new THREE.Vector3(0, 0, 0);
                break;
            case 'side':
                position = new THREE.Vector3(distance, 200, 0);
                target = new THREE.Vector3(0, 150, 0);
                break;
            case 'front':
                position = new THREE.Vector3(0, 200, distance);
                target = new THREE.Vector3(0, 150, 0);
                break;
            default: // reset
                position = new THREE.Vector3(500, 400, 500);
                target = new THREE.Vector3(0, 150, 0);
        }
        
        // Animate camera movement
        this.animateCamera(position, target);
    }
    
    animateCamera(targetPosition, targetLookAt) {
        const startPosition = this.camera.position.clone();
        const startTarget = this.controls.target.clone();
        const duration = 500;
        const startTime = Date.now();
        
        const animate = () => {
            const elapsed = Date.now() - startTime;
            const progress = Math.min(elapsed / duration, 1);
            const eased = this.easeOutCubic(progress);
            
            this.camera.position.lerpVectors(startPosition, targetPosition, eased);
            this.controls.target.lerpVectors(startTarget, targetLookAt, eased);
            this.controls.update();
            
            if (progress < 1) {
                requestAnimationFrame(animate);
            }
        };
        
        animate();
    }
    
    easeOutCubic(t) {
        return 1 - Math.pow(1 - t, 3);
    }
    
    onWindowResize() {
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;
        
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }
    
    // ===== Click Handling & Object Placement =====
    
    setupClickHandler() {
        this.renderer.domElement.addEventListener('dblclick', (event) => {
            this.onDoubleClick(event);
        });
        
        this.renderer.domElement.addEventListener('click', (event) => {
            this.onSingleClick(event);
        });
    }
    
    getMouseIntersection(event) {
        const rect = this.renderer.domElement.getBoundingClientRect();
        this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
        
        this.raycaster.setFromCamera(this.mouse, this.camera);
        
        // Check intersection with ground plane
        const intersects = this.raycaster.intersectObject(this.groundPlane);
        return intersects.length > 0 ? intersects[0].point : null;
    }
    
    onDoubleClick(event) {
        const point = this.getMouseIntersection(event);
        if (point) {
            this.placeObject(point.x, point.z);
        }
    }
    
    onSingleClick(event) {
        const rect = this.renderer.domElement.getBoundingClientRect();
        this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
        
        this.raycaster.setFromCamera(this.mouse, this.camera);
        
        // Check intersection with placed objects
        const intersects = this.raycaster.intersectObjects(this.placedObjects.map(o => o.mesh));
        
        if (intersects.length > 0) {
            this.selectObject(intersects[0].object);
        }
    }
    
    placeObject(x, z, width = 40, height = 60, depth = 40) {
        // Create rectangular box object
        const geometry = new THREE.BoxGeometry(width, height, depth);
        const material = new THREE.MeshStandardMaterial({
            color: this.colors.object,
            roughness: 0.5,
            metalness: 0.3
        });
        
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.set(x, height / 2, z);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        
        this.scene.add(mesh);
        
        const objectData = {
            mesh: mesh,
            position: { x, y: height / 2, z },
            dimensions: { width, height, depth },
            isInRange: this.checkObjectInRange(x, z)
        };
        
        this.placedObjects.push(objectData);
        this.selectObject(mesh);
        
        // Update UI to show object info
        this.updateObjectInfo(objectData);
        
        return objectData;
    }
    
    selectObject(mesh) {
        // Deselect previous
        if (this.selectedObject) {
            const prevObj = this.placedObjects.find(o => o.mesh === this.selectedObject);
            if (prevObj) {
                prevObj.mesh.material.color.setHex(
                    prevObj.isInRange ? this.colors.object : this.colors.objectOutOfRange
                );
            }
        }
        
        // Select new object
        this.selectedObject = mesh;
        const objData = this.placedObjects.find(o => o.mesh === mesh);
        
        if (objData) {
            mesh.material.color.setHex(this.colors.objectSelected);
            this.updateObjectInfo(objData);
        }
    }
    
    checkObjectInRange(x, z, objectHeight = 60) {
        const distance = Math.sqrt(x * x + z * z);
        // Check horizontal distance is within reach
        // Also check if the object is not too close (arm can't fold tightly enough)
        return distance >= this.minReach && distance <= this.maxReach;
    }
    
    updateMaxReach(dhParams) {
        // Calculate max reach based on DH parameters (link 2 + link 3)
        // This is the horizontal reach of the first 3 joints
        this.maxReach = dhParams.a2 + dhParams.a3 - 20; // Subtract margin
        this.minReach = Math.abs(dhParams.a2 - dhParams.a3) + 30; // Add margin
        
        // Update all objects' range status
        this.placedObjects.forEach(obj => {
            obj.isInRange = this.checkObjectInRange(obj.position.x, obj.position.z);
            if (obj.mesh !== this.selectedObject) {
                obj.mesh.material.color.setHex(
                    obj.isInRange ? this.colors.object : this.colors.objectOutOfRange
                );
            }
        });
    }
    
    updateObjectInfo(objData) {
        const infoEl = document.getElementById('object-info');
        if (infoEl) {
            const distance = Math.sqrt(objData.position.x ** 2 + objData.position.z ** 2).toFixed(1);
            infoEl.innerHTML = `
                <strong>Vật thể đã chọn:</strong><br>
                Vị trí: X=${objData.position.x.toFixed(1)}, Z=${objData.position.z.toFixed(1)}<br>
                Khoảng cách: ${distance}mm<br>
                Trạng thái: ${objData.isInRange ? '✅ Trong tầm với' : '❌ Ngoài tầm với'}
            `;
        }
    }
    
    getSelectedObjectPosition() {
        if (!this.selectedObject) return null;
        const objData = this.placedObjects.find(o => o.mesh === this.selectedObject);
        return objData ? objData.position : null;
    }
    
    isSelectedObjectInRange() {
        if (!this.selectedObject) return false;
        const objData = this.placedObjects.find(o => o.mesh === this.selectedObject);
        return objData ? objData.isInRange : false;
    }
    
    removeSelectedObject() {
        if (!this.selectedObject) return;
        
        const index = this.placedObjects.findIndex(o => o.mesh === this.selectedObject);
        if (index !== -1) {
            this.scene.remove(this.selectedObject);
            this.placedObjects.splice(index, 1);
            this.selectedObject = null;
            
            const infoEl = document.getElementById('object-info');
            if (infoEl) {
                infoEl.innerHTML = '<em>Double-click trên mặt phẳng để đặt vật thể</em>';
            }
        }
    }
    
    clearAllObjects() {
        this.placedObjects.forEach(obj => {
            this.scene.remove(obj.mesh);
        });
        this.placedObjects = [];
        this.selectedObject = null;
        
        const infoEl = document.getElementById('object-info');
        if (infoEl) {
            infoEl.innerHTML = '<em>Double-click trên mặt phẳng để đặt vật thể</em>';
        }
    }
    
    // Visual feedback when grabbing
    showGrabAnimation(success) {
        if (!this.selectedObject) return;
        
        const objData = this.placedObjects.find(o => o.mesh === this.selectedObject);
        if (!objData) return;
        
        if (success) {
            // Animation: lift the object
            const startY = objData.mesh.position.y;
            const targetY = startY + 100;
            const duration = 1000;
            const startTime = Date.now();
            
            const animateLift = () => {
                const elapsed = Date.now() - startTime;
                const progress = Math.min(elapsed / duration, 1);
                const eased = this.easeOutCubic(progress);
                
                objData.mesh.position.y = startY + (targetY - startY) * eased;
                
                if (progress < 1) {
                    requestAnimationFrame(animateLift);
                }
            };
            
            animateLift();
        }
    }
    
    // Show target marker for debugging
    showTargetMarker(x, y, z) {
        // Remove existing marker
        if (this.targetMarker) {
            this.scene.remove(this.targetMarker);
        }
        
        // Create a small sphere as target marker
        const geometry = new THREE.SphereGeometry(15, 16, 16);
        const material = new THREE.MeshStandardMaterial({
            color: 0xff00ff,  // Magenta for visibility
            emissive: 0xff00ff,
            emissiveIntensity: 0.5
        });
        this.targetMarker = new THREE.Mesh(geometry, material);
        this.targetMarker.position.set(x, y, z);
        this.scene.add(this.targetMarker);
        
        console.log(`Target marker placed at: X=${x}, Y=${y}, Z=${z}`);
    }
    
    // Get end effector fingertip position (for debugging)
    getGripperTipPosition() {
        if (this.jointMeshes.length === 0) return null;
        
        const endEffector = this.jointMeshes[this.jointMeshes.length - 1];
        if (!endEffector) return null;
        
        // End effector is a group, get world position
        const worldPos = new THREE.Vector3();
        endEffector.getWorldPosition(worldPos);
        
        // Fingertip is about 120mm below the end effector center
        // (matches the gripper design)
        const tipOffset = 120;
        
        // Get the direction the gripper is pointing
        const direction = new THREE.Vector3(0, -1, 0);
        direction.applyQuaternion(endEffector.quaternion);
        direction.multiplyScalar(tipOffset);
        
        return {
            x: worldPos.x + direction.x,
            y: worldPos.y + direction.y,
            z: worldPos.z + direction.z
        };
    }
    
    // ===== End Click Handling =====
    
    animate() {
        requestAnimationFrame(() => this.animate());
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}

// Initialize 3D visualization
let arm3D;
document.addEventListener('DOMContentLoaded', () => {
    arm3D = new Arm3DVisualization('canvas-container');
    
    // View control buttons
    document.getElementById('btn-reset-view').addEventListener('click', () => {
        arm3D.setCameraView('reset');
    });
    
    document.getElementById('btn-top-view').addEventListener('click', () => {
        arm3D.setCameraView('top');
    });
    
    document.getElementById('btn-side-view').addEventListener('click', () => {
        arm3D.setCameraView('side');
    });
    
    document.getElementById('btn-front-view').addEventListener('click', () => {
        arm3D.setCameraView('front');
    });
});
