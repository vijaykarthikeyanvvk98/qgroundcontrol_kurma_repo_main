#include "opencvimageprovider.h"

OpencvImageProvider::OpencvImageProvider(QObject *parent)
    : QQuickImageProvider(QQuickImageProvider::Image)
{
    image = QImage(200, 200, QImage::Format_RGB32);
    image.fill(QColor("black"));
}

QImage OpencvImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    Q_UNUSED(id);

    if (size) {
        *size = image.size();
    }

    if (requestedSize.width() > 0 && requestedSize.height() > 0) {
        image = image.scaled(requestedSize.width(), requestedSize.height(), Qt::KeepAspectRatio);
    }
    return image;
}

QRectF OpencvImageProvider::rectangle() const
{
    return m_rectangle;
}

void OpencvImageProvider::setRectangle(const QRectF &rect)
{

}

void OpencvImageProvider::updateImage(const QImage &image)
{
   // qDebug()<<"Image capturing";

    if (!image.isNull() && this->image != image) {
        this->image = image;
        emit imageChanged();
       // qDebug()<<"Image capturing";
    }

    else if (image.isNull()) {
        emit null_image_changed();
    }
}
