<?php
header('Access-Control-Allow-Origin: *');
header('Access-Control-Allow-Methods: POST, OPTIONS');
header('Access-Control-Allow-Headers: Content-Type');
header('Content-Type: application/json');
if ($_SERVER['REQUEST_METHOD'] === 'OPTIONS') { http_response_code(204); exit; }

// API-Key: in config.php oder direkt hier eintragen
$apiKey = '';
if (file_exists(__DIR__ . '/config_ai.php')) {
    include __DIR__ . '/config_ai.php';  // definiert $anthropicApiKey
    $apiKey = $anthropicApiKey ?? '';
}
if (!$apiKey) { http_response_code(400); echo json_encode(['error'=>'API-Key nicht konfiguriert. Bitte config_ai.php anlegen: <?php $anthropicApiKey="sk-ant-api03-...";']); exit; }

$body = json_decode(file_get_contents('php://input'), true);
if (empty($body['image'])) { http_response_code(400); echo json_encode(['error'=>'Kein Bild']); exit; }

$payload = [
    'model' => 'claude-sonnet-4-20250514',
    'max_tokens' => 1000,
    'messages' => [[
        'role' => 'user',
        'content' => [
            [
                'type' => 'image',
                'source' => [
                    'type' => 'base64',
                    'media_type' => $body['mime'] ?? 'image/png',
                    'data' => $body['image']
                ]
            ],
            [
                'type' => 'text',
                'text' => 'Dieses Bild zeigt einen Roboter mit Bemaßungen und kinematischen Daten (Endschalter). Extrahiere alle sichtbaren Maße und Achsgrenzen. Antworte NUR mit diesem JSON (kein Text davor/danach):\n{"axisOffsets":[{"name":"A1","x":0,"y":0,"z":0},{"name":"A2","x":0,"y":0,"z":0},{"name":"A3","x":0,"y":0,"z":0},{"name":"A4","x":0,"y":0,"z":0},{"name":"A5","x":0,"y":0,"z":0},{"name":"A6","x":0,"y":0,"z":0}],"axisLimits":[{"name":"J1","min":-170,"max":170},{"name":"J2","min":-65,"max":85},{"name":"J3","min":-180,"max":70},{"name":"J4","min":-300,"max":300},{"name":"J5","min":-130,"max":130},{"name":"J6","min":-360,"max":360}]}\nLeite die X/Y/Z Offsets aus den sichtbaren Bemaßungslinien ab. Alle Maße in mm.'
            ]
        ]
    ]]
];

$ch = curl_init('https://api.anthropic.com/v1/messages');
curl_setopt_array($ch, [
    CURLOPT_RETURNTRANSFER => true,
    CURLOPT_POST => true,
    CURLOPT_POSTFIELDS => json_encode($payload),
    CURLOPT_HTTPHEADER => [
        'Content-Type: application/json',
        'x-api-key: ' . $apiKey,
        'anthropic-version: 2023-06-01'
    ],
    CURLOPT_TIMEOUT => 30
]);
$result = curl_exec($ch);
$httpCode = curl_getinfo($ch, CURLINFO_HTTP_CODE);
curl_close($ch);

http_response_code($httpCode);
echo $result;
