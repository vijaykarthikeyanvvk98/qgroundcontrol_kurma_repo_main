#ifndef OPENCVIMAGEPROVIDER_H
#define OPENCVIMAGEPROVIDER_H

#include <QImage>
#include <QObject>
#include <QQuickImageProvider>

class OpencvImageProvider : public QQuickImageProvider
{
    Q_OBJECT
public:
    OpencvImageProvider(QObject *parent = nullptr);

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;

    Q_PROPERTY(QRectF rectangle READ rectangle NOTIFY rectangleChanged)
   public:
    QRectF m_rectangle;
    bool isdetected=false;
    QRectF rectangle() const;
    void setRectangle(const QRectF &rect);
public slots:
    void updateImage(const QImage &image);

signals:
    void imageChanged();
    void null_image_changed();
    void box_detected();
    void rectangleChanged();

private:
    QImage image;
};

#endif // OPENCVIMAGEPROVIDER_H
